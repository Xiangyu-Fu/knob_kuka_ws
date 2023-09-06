# Constants for the object recognition status used in ObjRec
uint8 OBJREC_STATUS_UNKNOWN = 0
uint8 OBJREC_STATUS_GOOD = 1
uint8 OBJREC_STATUS_BAD_POSE = 2 # wrong rotation of the object
uint8 OBJREC_STATUS_BAD_DISTANCE = 3 # object is too close to another object
