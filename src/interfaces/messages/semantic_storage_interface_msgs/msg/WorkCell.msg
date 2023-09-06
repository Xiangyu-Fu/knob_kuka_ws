#Described the workcell in a wrl file along with a list of the names of robots in the scence and their corresponding kinematic files
RosFile   scenegraph		# zipped wrl file describing entire scene
string[]  robot_semantic_names  # vector containing semantic names of robots in the scene
RosFile[] robot_kinematics	# vector containing corresponding robot kinematic models file contents (xml)

