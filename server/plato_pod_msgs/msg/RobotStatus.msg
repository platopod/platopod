# Status of a single registered robot
int32 robot_id              # unique sequential ID assigned at registration
string deployment           # "physical" or "virtual"
float64 x                   # metres from arena origin
float64 y                   # metres from arena origin
float64 theta               # heading in radians [-pi, pi]
float64 radius              # robot radius in metres
string status               # "active", "inactive", or "error"
string localization_id      # provider-specific ID (tag "3", "rover-1")
string localization_source  # "camera_artag", "gps_imu", "rf_anchor", etc.
