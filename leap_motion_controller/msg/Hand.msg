std_msgs/Header header

# A unique ID assigned to this Hand object
int32 lmc_hand_id

# How confident we are with a given hand pose (between 0.0 and 1.0)
float32 confidence

# The direction vector
geometry_msgs/Vector3 direction

# The normal vector
geometry_msgs/Vector3 normal

# The angle between the fingers and the hand of a grab hand pose. 
# In radians
float32 grab_strength

# The distance between the thumb and index finger of a pinch hand pose. 
float32 pinch_strength

# The palm pose (i.e., position and orientation of the center of the hand)
geometry_msgs/Pose palm_center

# The palm pose after applying an exponential smoothing filter
geometry_msgs/Pose palm_center_filtered

# A list of fingers and gestures associated with this hand
Finger[] finger_list