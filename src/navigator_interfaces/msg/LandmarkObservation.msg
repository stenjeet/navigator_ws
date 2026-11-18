# One landmark extracted from a lidar scan (course project stage 7).

# The observation is expressed in the base_footprint frame, i.e. it does not
# depend on the map pose estimate. This keeps it usable by the joint EKF-SLAM
# (stage 8), which estimates the pose and the landmarks together.

# Distance from the base_footprint origin to the landmark centre [m].
float64 range

# Angle of the landmark relative to the robot heading, in (-pi, pi] [rad].
float64 bearing

# Radius of the fitted circular landmark [m].
float64 radius

# 1-sigma variances of the two measurements.
float64 range_variance      # [m^2]
float64 bearing_variance    # [rad^2]
