# Navigation goal from external source (e.g., ATAK waypoint)
uint8 robot_id
float64 x           # arena-local metres
float64 y           # arena-local metres
string source_uid   # CoT UID of the originating event
builtin_interfaces/Time stamp
