# Pose of a single detected AprilTag in arena coordinates
int32 tag_id            # AprilTag ID
float64 x               # metres from Tag 101 (arena origin)
float64 y               # metres from Tag 101 (arena origin)
float64 theta           # heading in radians [-pi, pi]
float64 confidence      # detection quality 0.0-1.0 (from decision margin)
float64 timestamp       # detection timestamp (seconds)
