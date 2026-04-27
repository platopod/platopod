# Result of evaluating a FireIntent. Published by engagement_node.
#
# Consumed by api_gateway_node (broadcasts to WebSocket subscribers as
# engagement_event JSON), cot_bridge_node (emits CoT alerts and casualty
# markers to ATAK), and any analytics/recording sink.

int32 actor_id              # echoed from FireIntent
int32 target_id             # echoed
string weapon               # echoed
uint64 source_seq           # echoed correlation id

bool fired                  # false if blocked before evaluation (ROE, unknown weapon)
bool hit                    # true if PoK roll succeeded
bool blocked                # true if LoS or weather blocked the shot
float64 damage              # damage applied to target (0..1)
float64 distance_m          # actor → target distance in metres (or 0 for indirect)
float64 effective_pok       # post-cover, post-weather, post-fire-capability PoK

string rationale            # short machine-readable reason (e.g. "hit_clean",
                            # "miss_blocked_by_cover:wall", "out_of_range",
                            # "roe_blocked", "unknown_weapon")

bool civilian_violation     # true if a civilian was within proximity radius
string[] civilians_at_risk  # labels of nearby civilians

int32[] suppressed_targets  # robot_ids within suppression radius of impact
string[] roe_violations     # rule names from check_fire_roe (warnings + violations)
