"""Log-odds occupancy grid for the Navigator-M polygon (course project stage 6).

The grid is updated from 2D lidar scans and the robot pose:

* every valid beam marks the cells along the ray as free and the end cell as
  occupied (inverse sensor model in log-odds form);
* cells are clamped to ``[log_odds_min, log_odds_max]`` so a cell can still be
  flipped later and no measurement dominates forever;
* cells are published as occupied/free/unknown by converting log-odds back to
  a probability and applying the thresholds.

Ray traversal samples the beam with a step of ``resolution * step_ratio``
(half a cell by default) and deduplicates the touched cells, so the update is
vectorised over all beams and stays fast enough for 10 Hz scans in Python.

The module has no ROS dependencies (pure numpy) so it can be unit-tested; the
ROS wrapper lives in ``occupancy_grid_node.py``.
"""

import heapq
import math

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def log_odds(probability):
    """Convert an occupancy probability to log-odds."""
    value = float(probability)
    if not 0.0 < value < 1.0:
        raise ValueError(
            'probability must be in the open interval (0, 1), got %r' % value
        )
    return math.log(value / (1.0 - value))


def probability(log_odds_value):
    """Convert log-odds to an occupancy probability."""
    value = float(log_odds_value)
    if value >= 0.0:
        return 1.0 / (1.0 + math.exp(-value))
    exp_value = math.exp(value)
    return exp_value / (1.0 + exp_value)


def nearest_sample(times, stamp):
    """Index of the sample in ``times`` whose timestamp is closest to ``stamp``."""
    times = np.asarray(times, dtype=float)
    if times.size == 0:
        raise ValueError('cannot pick a sample from an empty list')
    return int(np.argmin(np.abs(times - float(stamp))))


class LogOddsGrid:
    """2D occupancy grid in log-odds form, indexed ``[iy, ix]``.

    Cell ``(ix, iy)`` covers ``[origin + index * resolution,
    origin + (index + 1) * resolution]``; its centre is at
    ``origin + (index + 0.5) * resolution``, which matches the way RViz renders
    ``nav_msgs/OccupancyGrid`` cells starting at ``info.origin``.
    """

    def __init__(
        self,
        width,
        height,
        resolution,
        origin=(0.0, 0.0),
        p_occupied=0.70,
        p_free=0.30,
        log_odds_min=-2.0,
        log_odds_max=3.5,
        occupied_threshold=0.60,
        free_threshold=0.40,
    ):
        width = int(width)
        height = int(height)
        resolution = float(resolution)
        if width < 1 or height < 1:
            raise ValueError('grid must be at least 1x1, got %dx%d' % (width, height))
        if resolution <= 0.0:
            raise ValueError('resolution must be positive, got %r' % resolution)

        log_odds_min = float(log_odds_min)
        log_odds_max = float(log_odds_max)
        if not log_odds_min < 0.0 < log_odds_max:
            raise ValueError('log-odds clamp must straddle zero')

        occupied_threshold = float(occupied_threshold)
        free_threshold = float(free_threshold)
        if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
            raise ValueError(
                'thresholds must satisfy 0 <= free < occupied <= 1, got %r, %r'
                % (free_threshold, occupied_threshold)
            )

        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = (float(origin[0]), float(origin[1]))
        self.p_occupied = float(p_occupied)
        self.p_free = float(p_free)
        self.log_odds_min = log_odds_min
        self.log_odds_max = log_odds_max
        self.occupied_threshold = occupied_threshold
        self.free_threshold = free_threshold

        self.hit_delta = log_odds(self.p_occupied)
        self.miss_delta = log_odds(self.p_free)
        self.log_odds = np.zeros((height, width), dtype=float)

    # ------------------------------------------------------------------
    # Constructors and geometry helpers
    # ------------------------------------------------------------------
    @classmethod
    def from_bounds(cls, min_x, min_y, max_x, max_y, resolution, **kwargs):
        """Grid covering ``[min_x, max_x) x [min_y, max_y)`` with cell centres
        anchored at the corner ``(min_x, min_y)``."""
        resolution = float(resolution)
        if max_x <= min_x or max_y <= min_y:
            raise ValueError('max bounds must exceed min bounds')
        width = int(math.ceil((float(max_x) - float(min_x)) / resolution))
        height = int(math.ceil((float(max_y) - float(min_y)) / resolution))
        return cls(width, height, resolution, origin=(min_x, min_y), **kwargs)

    def world_to_grid(self, x, y, clip=False):
        """Cell indices ``(ix, iy)`` of a world point, or ``None`` if outside."""
        ix = int(math.floor((float(x) - self.origin[0]) / self.resolution))
        iy = int(math.floor((float(y) - self.origin[1]) / self.resolution))
        if clip:
            ix = min(max(ix, 0), self.width - 1)
            iy = min(max(iy, 0), self.height - 1)
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            return None
        return ix, iy

    def grid_to_world(self, ix, iy):
        """World coordinates of a cell centre."""
        return (
            self.origin[0] + (int(ix) + 0.5) * self.resolution,
            self.origin[1] + (int(iy) + 0.5) * self.resolution,
        )

    def occupancy_at(self, x, y):
        """Published cell value at a world point (``None`` when outside)."""
        indices = self.world_to_grid(x, y)
        if indices is None:
            return None
        data = self.occupancy_data()
        return int(data[indices[1], indices[0]])

    # ------------------------------------------------------------------
    # Mapping
    # ------------------------------------------------------------------
    def update_scan(
        self,
        pose,
        ranges,
        angle_min,
        angle_increment,
        range_min=0.0,
        range_max=float('inf'),
        lidar_offset=(0.0, 0.0, 0.0),
        max_range=None,
        step_ratio=0.5,
    ):
        """Update the grid from one lidar scan.

        ``pose`` is ``(x, y, yaw)`` of ``base_footprint`` in the map frame and
        ``lidar_offset`` is the lidar mounting offset in ``base_link`` (the
        ``z`` component is ignored for the 2D map). Non-finite ranges and
        ranges outside ``[range_min, range_max]`` (or above ``max_range``) are
        skipped, so lidar dropouts (``inf``) do not affect the map.

        Returns ``(free_cells, occupied_cells)`` touched by this scan.
        """
        x, y, yaw = (float(v) for v in pose)
        ranges = np.asarray(ranges, dtype=float).reshape(-1)

        valid = np.isfinite(ranges) & (ranges > 0.0)
        valid &= ranges >= float(range_min)
        valid &= ranges <= float(range_max)
        if max_range is not None:
            valid &= ranges <= float(max_range)

        beams = np.flatnonzero(valid)
        if beams.size == 0:
            return 0, 0

        distances = ranges[beams]
        angles = yaw + float(angle_min) + beams * float(angle_increment)

        dx, dy, _ = (float(v) for v in lidar_offset)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        lidar_x = x + dx * cos_yaw - dy * sin_yaw
        lidar_y = y + dx * sin_yaw + dy * cos_yaw
        dir_x, dir_y = np.cos(angles), np.sin(angles)

        # Free cells: sample the ray and stop one step short of the end point,
        # which belongs to the obstacle surface. A step of half a cell keeps
        # the number of skipped cells negligible.
        step = max(self.resolution * float(step_ratio), 1e-3)
        n_steps = int(math.ceil(float(distances.max()) / step)) + 1
        travelled = np.arange(n_steps, dtype=float) * step
        points_x = lidar_x + np.outer(dir_x, travelled)
        points_y = lidar_y + np.outer(dir_y, travelled)
        free_mask = travelled[None, :] <= (distances[:, None] - step)

        ix = np.floor((points_x - self.origin[0]) / self.resolution).astype(np.int64)
        iy = np.floor((points_y - self.origin[1]) / self.resolution).astype(np.int64)
        inside = (
            (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        )
        flat_free = np.unique((iy * self.width + ix)[free_mask & inside])

        end_x = lidar_x + distances * dir_x
        end_y = lidar_y + distances * dir_y
        end_ix = np.floor((end_x - self.origin[0]) / self.resolution).astype(np.int64)
        end_iy = np.floor((end_y - self.origin[1]) / self.resolution).astype(np.int64)
        end_inside = (
            (end_ix >= 0)
            & (end_ix < self.width)
            & (end_iy >= 0)
            & (end_iy < self.height)
        )
        flat_occupied = np.unique(
            (end_iy * self.width + end_ix)[end_inside]
        )

        # A cell hit by one beam and merely crossed by another is a surface:
        # the occupied evidence wins for this scan.
        flat_free = np.setdiff1d(flat_free, flat_occupied, assume_unique=True)

        cells = self.log_odds.reshape(-1)
        cells[flat_free] += self.miss_delta
        cells[flat_occupied] += self.hit_delta
        np.clip(
            self.log_odds, self.log_odds_min, self.log_odds_max, out=self.log_odds
        )
        return int(flat_free.size), int(flat_occupied.size)

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------
    def occupancy_data(self):
        """Map as ``+100/0/-1`` integers, shape ``(height, width)``."""
        data = np.full(self.log_odds.shape, UNKNOWN, dtype=np.int8)
        occupied = self.probability_data()
        data[occupied >= self.occupied_threshold] = OCCUPIED
        data[occupied <= self.free_threshold] = FREE
        return data

    def probability_data(self):
        """Occupancy probability of every cell, shape ``(height, width)``."""
        result = np.empty_like(self.log_odds)
        positive = self.log_odds >= 0.0
        result[positive] = 1.0 / (1.0 + np.exp(-self.log_odds[positive]))
        exp_values = np.exp(self.log_odds[~positive])
        result[~positive] = exp_values / (1.0 + exp_values)
        return result

    def stats(self):
        """Counts of occupied, free and unknown cells."""
        data = self.occupancy_data()
        return {
            'occupied': int(np.count_nonzero(data == OCCUPIED)),
            'free': int(np.count_nonzero(data == FREE)),
            'unknown': int(np.count_nonzero(data == UNKNOWN)),
        }


def distance_to_occupied(occupied, resolution=1.0):
    """Distance from every cell to the nearest occupied cell, in metres.

    Multi-source Dijkstra on the 8-connected grid (diagonal step ``sqrt(2)``
    cells). Cells that cannot reach any occupied cell get ``inf``.
    """
    occupied = np.asarray(occupied, dtype=bool)
    if occupied.ndim != 2:
        raise ValueError('occupancy mask must be 2D')
    distances = np.full(occupied.shape, np.inf, dtype=float)
    if not occupied.any():
        return distances

    height, width = occupied.shape
    diagonal = math.sqrt(2.0)
    queue = []
    for iy, ix in np.argwhere(occupied):
        iy, ix = int(iy), int(ix)
        distances[iy, ix] = 0.0
        queue.append((0.0, iy, ix))
    heapq.heapify(queue)

    while queue:
        current, iy, ix = heapq.heappop(queue)
        if current > distances[iy, ix]:
            continue
        for dy in (-1, 0, 1):
            ny = iy + dy
            if not 0 <= ny < height:
                continue
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = ix + dx
                if not 0 <= nx < width:
                    continue
                candidate = current + (diagonal if dx and dy else 1.0)
                if candidate < distances[ny, nx]:
                    distances[ny, nx] = candidate
                    heapq.heappush(queue, (candidate, ny, nx))

    return distances * float(resolution)


def compare_occupancy_data(data_a, data_b, resolution=1.0):
    """Compare two ``-1/0/100`` maps of equal shape.

    Returns occupied-cell IoU, cell-level agreement over the cells known in
    both maps, the raw counts and the chamfer-style distance from the occupied
    cells of one map to the nearest occupied cell of the other (metres when
    ``resolution`` is given in metres per cell). ``only_a`` counts cells
    occupied in ``a`` but not in ``b``; ``only_b`` is the reverse.
    """
    a = np.asarray(data_a)
    b = np.asarray(data_b)
    if a.shape != b.shape:
        raise ValueError('maps must have the same shape: %s vs %s' % (a.shape, b.shape))

    occupied_a = a == OCCUPIED
    occupied_b = b == OCCUPIED
    union = int(np.count_nonzero(occupied_a | occupied_b))
    both = int(np.count_nonzero(occupied_a & occupied_b))
    known = (a != UNKNOWN) & (b != UNKNOWN)
    n_known = int(np.count_nonzero(known))

    distance_to_b = distance_to_occupied(occupied_b, resolution)
    distance_to_a = distance_to_occupied(occupied_a, resolution)
    distances_ab = distance_to_b[occupied_a]
    distances_ba = distance_to_a[occupied_b]

    return {
        'iou_occupied': float(both) / union if union else float('nan'),
        'agreement': (
            float(np.count_nonzero((a == b) & known)) / n_known
            if n_known else float('nan')
        ),
        'occupied_a': int(np.count_nonzero(occupied_a)),
        'occupied_b': int(np.count_nonzero(occupied_b)),
        'both_occupied': both,
        'only_a': int(np.count_nonzero(occupied_a & ~occupied_b)),
        'only_b': int(np.count_nonzero(occupied_b & ~occupied_a)),
        'known_both': n_known,
        'mean_distance_ab': (
            float(np.mean(distances_ab)) if distances_ab.size else float('nan')
        ),
        'max_distance_ab': (
            float(np.max(distances_ab)) if distances_ab.size else float('nan')
        ),
        'mean_distance_ba': (
            float(np.mean(distances_ba)) if distances_ba.size else float('nan')
        ),
        'max_distance_ba': (
            float(np.max(distances_ba)) if distances_ba.size else float('nan')
        ),
    }


def compare_grids(grid_a, grid_b):
    """``compare_occupancy_data`` for two grids with the same geometry."""
    if (grid_a.width, grid_a.height) != (grid_b.width, grid_b.height):
        raise ValueError(
            'grid shapes differ: %dx%d vs %dx%d'
            % (grid_a.width, grid_a.height, grid_b.width, grid_b.height)
        )
    if not math.isclose(grid_a.resolution, grid_b.resolution):
        raise ValueError(
            'resolutions differ: %r vs %r'
            % (grid_a.resolution, grid_b.resolution)
        )
    if grid_a.origin != grid_b.origin:
        raise ValueError('origins differ: %r vs %r' % (grid_a.origin, grid_b.origin))
    return compare_occupancy_data(
        grid_a.occupancy_data(), grid_b.occupancy_data(),
        resolution=grid_a.resolution,
    )
