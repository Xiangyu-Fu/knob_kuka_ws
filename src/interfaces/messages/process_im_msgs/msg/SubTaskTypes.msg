#sub task types
# -----------------------
# WARNING: If you change definitions in here, make sure to change them also for JavaScript:
# gui_process_editor/ros_rope/server/public/js/TaskActionTypes.js
# -----------------------

int32 NONE					= 0
int32 GRASP 				= 70001
int32 PLACE 				= 70002
int32 PICK_PLACE 			= 70003
int32 PICK_PLACE_ENDPOSE 		= 70004
int32 PICK_PLACE_APPROACH_ENDPOSE 	= 70005
int32 PUSH_DOWN 			= 70006
int32 MOVE	 			= 70007
int32 MOVE_TO_ACTOR 			= 70008
int32 WELD_SPOT				= 70009
int32 WELD_SEAM 			= 70010
int32 MOVE_HOME				= 70011
int32 DETACH_TOOL 			= 70012
int32 GRASP_RELEASE			= 70013
int32 NAIL 		= 70015
int32 SAW 		= 70016
int32 DISASSEMBLE_ENDPOSE 		= 70017
int32 DISASSEMBLE_APPROACH_ENDPOSE 		= 70018

int32 DUAL_ARM_MOVE                          = 71001
int32 DUAL_ARM_MOVE_OBJECT                   = 71002
int32 DUAL_ARM_PUSH                          = 71003
int32 DUAL_ARM_GRASP                         = 71004
int32 DUAL_ARM_PLACE                         = 71005
int32 DUAL_ARM_PICK_PLACE                    = 71006
int32 DUAL_ARM_PICK_PLACE_END_POSE           = 71007
int32 DUAL_ARM_PICK_PLACE_APPROACH_END_POSE  = 71008
int32 DUAL_ARM_ASSEMBLY_AIR_END_POSE 	     = 71009 
int32 DUAL_ARM_ASSEMBLY_GROUND_END_POSE      = 71010 
int32 DUAL_ARM_MOVE_ACTOR                    = 71011
int32 DUAL_ARM_GRASP_RELEASE                 = 71012
