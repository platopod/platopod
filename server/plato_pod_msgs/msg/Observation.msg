# Observation report from an actor about a target.
#
# Used to clear targets for ROE under WEAPONS TIGHT (classification == "hostile"
# adds the target to the engagement_node's cleared set). Published by
# api_gateway_node when a player sends a report_observation WebSocket message.

int32 actor_id              # robot_id of the observing unit
int32 target_id             # robot_id of the observed unit
string classification       # "hostile" | "friendly" | "civilian" | "unknown"
float64 confidence          # 0..1; 1.0 if not provided
