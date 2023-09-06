#parameters associated with pick and place with approach and  end pose

geometry_msgs/Pose grasp_approach
geometry_msgs/Pose grasp
float32 gripper_span
bool grasp_from_inside

geometry_msgs/Pose place_approach_approach
geometry_msgs/Pose place_approach
geometry_msgs/Pose place
geometry_msgs/Pose place_post_approach

geometry_msgs/Pose pickobject_final_pose #final pose of the OBJECT

# Only used for D2 workcell.
float64 grasp_approach_speed
float64 grasp_speed
float64 place_approach_approach_speed
float64 place_approach_speed
float64 place_speed
float64 place_post_approach_speed

geometry_msgs/Pose grasp_close_approach
geometry_msgs/Pose measurement_home_approach
geometry_msgs/Pose measurement_home
geometry_msgs/Pose measure_y
geometry_msgs/Pose measure_x
geometry_msgs/Pose orient_panel

float64 grasp_close_approach_speed
float64 measurement_home_approach_speed
float64 measurement_home_speed
float64 measure_y_speed
float64 measure_x_speed
float64 orient_panel_speed

float64 width
float64 height
float64 depth
