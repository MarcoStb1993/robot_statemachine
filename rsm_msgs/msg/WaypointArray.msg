# Describes an array of waypoints to follow
std_msgs/Header header
string ns

uint8 mode              # Waypoint following mode

uint8 SINGLE = 0        # Single trip from start to end
uint8 ROUNDTRIP = 1     # Drive from start to end then to start again and repeat
uint8 PATROL = 2        # Drive from start to end then in reverse and repeat

bool reverse            # Driving from start to end (=false) or end to start (=true)

Waypoint[] waypoints    # Array of all waypoints for following
int32 waypoints_size     # Number of waypoints in the waypoint array
