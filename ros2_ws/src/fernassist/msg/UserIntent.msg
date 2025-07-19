# User Intent Message
# Represents the processed intent from user AAC input

string raw_text               # Original text from AAC input
float32 confidence           # Confidence score of the intent interpretation
time timestamp               # Timestamp of intent processing
string intent_type           # Type of intent (e.g., "command", "question", "statement")
string[] entities            # Extracted entities from the text 