# AAC Input Message
# Represents input from Augmentative and Alternative Communication devices

string text                    # The text input from the AAC device
float32 confidence            # Confidence score (0.0-1.0) of the input
string input_type             # Type of input (e.g., "text", "symbol", "gesture")
time timestamp                # Timestamp of when the input was received
string device_id              # Identifier for the AAC device 