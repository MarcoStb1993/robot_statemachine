#Operation mode including information about emergency stop
bool emergencyStop					# Is emergency stop activated

uint8 STOPPED = 0        			# Robot is stopped
uint8 AUTONOMOUS = 1     				# Robot is controlled by the autonomy cmd vel topic	
uint8 TELEOPERATION = 2        		# Robot is controlled by the teleoperation cmd vel topic	

uint8 mode							# Operation mode
