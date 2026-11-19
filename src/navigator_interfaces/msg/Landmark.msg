# One landmark in the map frame (course project stages 7-8).

# Identifier assigned by the association step. Stable for the whole run.
int32 id

float64 x
float64 y

# 2x2 position covariance, row-major: [xx, xy, yx, yy] [m^2].
float64[4] covariance
