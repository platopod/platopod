# Generic envelope for any sensor reading.
#
# Replaces `std_msgs/String + JSON` on /robot_<id>/sensors/<name>.
#
# A consumer in any language can:
#   * Read robot_id, sensor_name, timestamp, schema_version, payload_format
#     directly from the typed message — no parsing required for routing.
#   * Decode payload only when it cares about that sensor type, using its
#     language's native JSON/CBOR/MsgPack library.
#
# When a sensor type stabilises and its schema is hot-pathed (thermal, lidar,
# rangefinder), a parallel typed message can be published alongside; consumers
# pick whichever is convenient. The envelope remains the source of truth
# for routing / archiving / replay.

int32 robot_id              # owning robot
string sensor_name          # registered plugin name (gas, thermal, lidar_2d, …)
float64 timestamp           # unix epoch seconds
string schema_version       # sensor-specific version of the payload schema

string payload_format       # "json" (default) | "cbor" | "msgpack"
string payload              # serialized payload in the declared format

# Optional typed echo: when true, a parallel typed message has been published
# on /robot_<id>/sensors/<name>/typed for this reading. Consumers that prefer
# strong typing can subscribe to that topic and ignore `payload`.
bool typed_published
