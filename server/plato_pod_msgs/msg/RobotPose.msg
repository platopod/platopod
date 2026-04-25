# A robot's pose from any localization backend
int32 robot_id              # unique robot identifier
float64 x                   # position in metres, arena-local frame
float64 y                   # position in metres, arena-local frame
float64 theta               # heading in radians, 0 = arena +x axis
float64 timestamp           # seconds since epoch
string source               # localization backend: "camera_artag", "gps_imu", "rf_anchor", "virtual_sim"
float32 confidence          # 0.0 to 1.0, method-dependent
string localization_id      # provider-specific ID (tag "3", "rover-1", etc.)
