# A scoring zone within the arena, stored as a polygon
string name                 # zone identifier
string team                 # team that scores here (empty string if any team)
float64[] vertices_x        # polygon X coordinates (metres from origin)
float64[] vertices_y        # polygon Y coordinates (metres from origin)
float64 hold_time_seconds   # time a robot must remain in zone to score
string original_type        # "rectangle", "polygon", or "circle"
