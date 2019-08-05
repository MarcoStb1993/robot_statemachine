#A waypoint for waypoint following
geometry_msgs/Pose pose     # Pose of this waypoint
bool visited                # True if the waypoint was already visited
bool unreachable			# True if no path to this waypoint could be found
string routine				# Routine to be executed on reaching waypoint