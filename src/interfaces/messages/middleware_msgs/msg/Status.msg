bool   success # True signals a success. False signals an error.
int32  level   # Importance level of this status. See middleware_msgs/StatusLevelsEnum.msg.
int32  code    # The status code of this status. See the documentation of each node.
string message # A dynamically-generated, human-readable, English status message that can be presented to the user.
