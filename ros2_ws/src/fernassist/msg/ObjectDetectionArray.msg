# Object Detection Array Message for FernAssist

# Header with timestamp and frame information
std_msgs/Header header

# Array of detected objects
DetectedObject[] detections

# Detection statistics
int32 total_detections
float64 processing_time
float64 confidence_threshold 