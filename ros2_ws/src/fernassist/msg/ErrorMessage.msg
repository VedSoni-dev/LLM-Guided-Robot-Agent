# Error Message for FernAssist

# Error identification
string error_code
string error_message
string severity  # ERROR, WARNING, INFO

# Context information
string context  # JSON string with additional context
time timestamp

# Recovery information
bool auto_recovery_enabled
int32 retry_count
float64 timeout_duration 