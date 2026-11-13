# Request from vision/navigation to the speech subsystem.
std_msgs/Header header
string text
# 0 = normal, 1 = important, 2 = safety
uint8 priority
string reason
string language
