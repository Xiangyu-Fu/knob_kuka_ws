float32 width
float32 force
float32 acceleration
int32 gripper_state
	int32 GRASPING_STATE_IDLE = 0
	int32 GRASPING_STATE_GRASPING = 1
	int32 GRASPING_STATE_NO_PART_FOUND =2
	int32 GRASPING_STATE_PART_LOST = 3
	int32 GRASPING_STATE_HOLDING = 4
	int32 GRASPING_STATE_RELEASING = 5
	int32 GRASPING_STATE_POSITIONING = 6
	int32 GRASPING_STATE_ERROR = 7