# Similar to nav_msgs/Path.msg, but has two paths (blue and yellow)
# Positions are relative to the vehicle, not global
Header header
geometry_msgs/PoseStamped[] blue_cones
geometry_msgs/PoseStamped[] yellow_cones
std_msgs/Int16 size
