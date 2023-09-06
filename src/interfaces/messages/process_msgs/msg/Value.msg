uint8 valueEnum	     				# see process_msgs/TypesEnum

bool[] b
int64[] l
float64[] d
string[] s

Individual[] individual # Examples: panel material is Powerpanel HD
PhysicalEntity[] physicalEntity # Examples: pickObject is ball bearing 2, placeObject is mechanical pipe 1
CombinedGeometricInterrelationConstraint[] combinedGeometricInterrelationConstraint # Examples: pose constraints from CAD mating GUI
geometry_msgs/Pose[] pose # Examples: The final pose of a disassembled object is its starting pose at the beginning of an execution.
