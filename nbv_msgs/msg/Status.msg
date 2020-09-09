# Status of nbv planner. This status show state of work

int8 STATUS_WAIT =  0        	# wait for initialization
int8 STATUS_RUN =  1        	# work
int8 STATUS_RESET = 2 		# reset planner and maps
int8 STATUS_STOP = 3       	# Stop
int8 STATUS_COMPLETE = 4        # exploration complete
int8 STATUS_GO_HOME = 5        	# go to the initial position
int8 STATUS_GO_HOME_COMPLETE = 6    # drone is in the initial position

int8 status
