int32 UNDEFINED =     0 # Default.
int32 INCOMPLETE = 10000 # The user has not yet set all non-optional parameters.
int32 VALID = 20000 # The user has set all process and task parameters correctly in the GUI.
int32 INVALID = 30000 # The user has set some task parameters incorrectly.
int32 EXECUTABLE = 40000 # Idle. Abstract process has been successfully mapped to a workcell and the task is waiting to be executed.
int32 EXECUTING = 50000
int32 COMPLETED = 60000 # Success.
int32 FAILED = 70000 # Fail.
int32 STOPPED = 80000 # Aborted.
