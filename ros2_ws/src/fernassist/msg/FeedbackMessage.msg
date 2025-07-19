# Feedback Message for FernAssist

# Message type
string message_type  # success, error, warning, info, progress, confirmation

# Content
string title
string content

# TTS information
string tts_text
string voice
int32 rate

# Visual feedback
string visual_icon
string color
float64 duration

# Metadata
int32 priority
bool requires_confirmation
string error_code
time timestamp 