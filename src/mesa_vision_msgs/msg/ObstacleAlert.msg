# Result of confirming a camera detection against LiDAR / depth.
std_msgs/Header header
string label
float32 distance_m
float32 bearing_rad
geometry_msgs/Point position
bool confirmed
# "lidar", "depth_only" or "none"
string confirmation_source
bool is_person
bool movable
# "monitor", "replan", "yield" or "ask_to_move"
string recommended_action
