# Complete arena environment model: boundary + obstacles + zones
float64[] boundary_x                  # boundary polygon X coordinates (metres, CCW order)
float64[] boundary_y                  # boundary polygon Y coordinates (metres, CCW order)
plato_pod_msgs/Obstacle[] obstacles   # static obstacles
plato_pod_msgs/Zone[] zones           # scoring zones
int32 origin_tag                      # origin tag ID (default 101)
float64 timestamp                     # model update timestamp (seconds)
bool boundary_valid                   # true if boundary computed from tags
