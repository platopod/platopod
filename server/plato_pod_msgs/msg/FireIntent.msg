# A fire intent: an actor wants to engage a target with a weapon.
# Published by opfor_node (autonomous OPFOR) and api_gateway_node
# (player-initiated). Consumed by engagement_node.
#
# Either target_id >= 0 (engage a specific unit) or has_target_position == true
# (engage a position, e.g. indirect fire). actor_id == -1 indicates no actor
# (indirect fire / artillery / CAS).

int32 actor_id              # robot_id of the firing unit; -1 for indirect fire
int32 target_id             # robot_id of the target; -1 if position-only
string weapon               # weapon name; must exist in the weapon catalog

bool has_target_position    # true → use target_x/y; false → require target_id
float64 target_x            # arena metres (ignored if has_target_position == false)
float64 target_y

uint64 source_seq           # opaque correlation id; echoed in EngagementOutcome
