# 3x3 homography matrix mapping camera pixels to arena metres
float64[9] matrix       # row-major 3x3 homography
bool valid               # true if computed from all 4 boundary corners
float64 timestamp        # computation timestamp (seconds)
