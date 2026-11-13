# One object detected by the OAK-D Lite (neural network + stereo depth).
# Position is in the camera optical frame: x right, y down, z forward (metres).
std_msgs/Header header
string label
float32 confidence
geometry_msgs/Point position
float32 distance_m
# Bearing in the robot yaw convention (REP-103): positive = to the left, radians.
float32 bearing_rad
# Normalised bounding box in the RGB image (0..1).
float32 xmin
float32 ymin
float32 xmax
float32 ymax
# Set by person_tracker_node (0 = not tracked).
int32 track_id
float32 speed_mps
