string typeIri					# required
string iri					# optional, for example set by an object detector with tracking
geometry_msgs/PoseWithCovarianceStamped pose
float64 poseAccuracy # For example: 0.01 (m)
float64 confidence # between 0 and 1 on the confidence of the detected object
uint8 status # object status from ObjRecStatus.msg
