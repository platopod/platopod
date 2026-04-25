# All tag detections from a single camera frame
plato_pod_msgs/TagPose[] tags            # all detected tags this frame
plato_pod_msgs/TagPose[] boundary_tags   # only boundary tags (IDs 100-103)
plato_pod_msgs/TagPose[] robot_tags      # only robot tags (IDs 0-15)
bool origin_visible                       # is Tag 101 currently detected?
int64 frame_id                            # sequential frame counter
float64 fps                               # current detection rate
