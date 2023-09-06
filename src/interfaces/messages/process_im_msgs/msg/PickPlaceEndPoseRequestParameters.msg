#request parameters associated with pick and place with only end pose

# D2 workcell
bool use_laser_distance_d2
float64 laser_distance_d2

bool move_measure_d2

bool use_measures_d2
float64 measure_x1_d2
float64 measure_x2_d2
float64 measure_y1_d2

bool afterMoveMeasureYPoseIsValid
geometry_msgs/Pose afterMoveMeasureYPose

bool afterMoveMeasureXPoseIsValid
geometry_msgs/Pose afterMoveMeasureXPose
