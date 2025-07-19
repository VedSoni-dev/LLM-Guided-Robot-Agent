# Robot Action Message
# Represents a command for the robot to execute

string action_type            # Type of action (e.g., "MOVE", "GRAB", "PLACE", "SPEAK")
string parameters             # JSON string containing action parameters
float32 confidence           # Confidence score of the action
string original_text          # Original user input that led to this action
time timestamp               # Timestamp of action generation
string status                # Current status of the action 