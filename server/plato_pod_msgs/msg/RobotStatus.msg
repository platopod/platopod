# Status of a single registered robot
int32 robot_id              # unique sequential ID assigned at registration
string deployment           # "physical" or "virtual"
float64 x                   # metres from arena origin
float64 y                   # metres from arena origin
float64 theta               # heading in radians [-pi, pi]
float64 radius              # robot radius in metres
string status               # "active", "inactive", "error", "wounded", "destroyed", "incapacitated", "frozen"
string localization_id      # provider-specific ID (tag "3", "rover-1")
string localization_source  # "camera_artag", "gps_imu", "rf_anchor", etc.

# Tactical state (extended for AR military exercises; defaults preserve back-compat)
string team                 # "blue", "red", "green", or "" (free-play)
string vehicle_role         # "tank", "recon", "apc", "soldier", "civilian", "hostile", "default"
float64 health              # 0.0..1.0; 0=destroyed, 1=full health
string[] weapons            # weapon names available to this unit
float64 thermal_signature   # 0.0..1.0; for thermal sensor twin
