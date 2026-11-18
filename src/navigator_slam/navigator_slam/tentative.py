"""Confirmation buffer for unmatched landmark observations (stages 7-8).

A real landmark is re-observed hundreds of times along the trajectory, while a
short arc on a wall corner or a wall edge is reported a handful of times (the
circle fit is ill-conditioned for three collinear points, so a corner looks
like a small cylinder). Creating a map entry from the first sighting therefore
leaves phantom landmarks behind: the offline run of the stage 7 -> 8 pipeline
gave 9 landmarks for 4 real ones, with 263..414 sightings per real landmark and
1..5 per phantom.

``TentativeTracks`` keeps such candidates in a small buffer and only confirms
them after ``min_hits`` consistent sightings within ``gate`` metres.
"""

import math


class TentativeTracks:
    """Buffer of candidate landmarks waiting for confirmation."""

    def __init__(self, gate=0.35, min_hits=5, max_tracks=200):
        gate = float(gate)
        min_hits = int(min_hits)
        max_tracks = int(max_tracks)
        if gate <= 0.0:
            raise ValueError('tentative gate must be positive, got %r' % gate)
        if min_hits < 1:
            raise ValueError('min_hits must be at least 1, got %r' % min_hits)
        if max_tracks < 1:
            raise ValueError('max_tracks must be at least 1, got %r' % max_tracks)

        self.gate = gate
        self.min_hits = min_hits
        self.max_tracks = max_tracks
        #: ``[x, y, hits]`` of every candidate, oldest first.
        self.tracks = []

    def __len__(self):
        return len(self.tracks)

    def positions(self):
        """Current positions of the candidates (for logging/tests)."""
        return [(track[0], track[1]) for track in self.tracks]

    def update(self, x, y):
        """Register one sighting of a candidate at ``(x, y)``.

        The sighting is matched to the nearest candidate within ``gate`` (an
        exponential moving average keeps its position); otherwise a new
        candidate is appended. Returns ``(confirmed, x, y)``: ``confirmed`` is
        true when the candidate has been seen ``min_hits`` times, and the
        returned position is then the averaged one. With ``min_hits == 1``
        every sighting is confirmed immediately (no buffering).
        """
        x = float(x)
        y = float(y)
        if self.min_hits <= 1:
            return True, x, y

        best = None
        best_distance = self.gate
        for track in self.tracks:
            distance = math.hypot(track[0] - x, track[1] - y)
            if distance <= best_distance:
                best, best_distance = track, distance

        if best is None:
            self.tracks.append([x, y, 1])
            if len(self.tracks) > self.max_tracks:
                self.tracks.pop(0)
            return False, x, y

        best[0] = 0.5 * (best[0] + x)
        best[1] = 0.5 * (best[1] + y)
        best[2] += 1
        if best[2] >= self.min_hits:
            self.tracks.remove(best)
            return True, best[0], best[1]
        return False, best[0], best[1]

    def clear(self):
        """Drop every candidate."""
        self.tracks = []
