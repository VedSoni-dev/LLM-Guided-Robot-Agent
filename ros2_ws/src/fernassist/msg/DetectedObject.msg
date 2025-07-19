# Detected Object Message for FernAssist

# Object classification
int32 class_id
string class_name
float64 confidence

# 2D bounding box [x1, y1, x2, y2]
float64[] bbox

# 3D position in camera frame (meters)
geometry_msgs/Point position

# 3D size estimates (width, height, depth in meters)
geometry_msgs/Point size

# Tracking information
int32 track_id

# Timestamp
time timestamp

# Additional metadata
string[] attributes 