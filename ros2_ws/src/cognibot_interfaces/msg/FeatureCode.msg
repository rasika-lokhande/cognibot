# Represents a single spatial feature that bridges perception and action
string feature_name          # e.g., "ForwardClear", "RightOpen", "LeftWall"
float32 activation_level     # Current activation strength (0.0 to 1.0)
float32 perceptual_evidence  # How strongly perception supports this feature
float32 action_bias          # How much this feature biases action selection
float32 task_weight          # Attentional weighting based on current task