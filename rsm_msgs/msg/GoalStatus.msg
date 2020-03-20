#Information about currently active goal
geometry_msgs/Pose goal			 	# Goal in the map
uint8 goal_status					# Status of the goal 

uint8 ACTIVE = 0       				# Goal is still active
uint8 REACHED = 1       			# Goal was successfully reached
uint8 FAILED = 2        			# Failed to reach goal
uint8 ABORTED = 3        			# Aborted goal before completion