# Rocket-related log messages

float32 input_altitude			# Altitude given to controller (meters)
float32 input_velocity			# Velocity given to controller (meters/second)
float32 apogee_estimate		# Estimated apogee altitude (meters)
float32 error				# Error - difference between target apogee and estimated apogee (meters)
float32 target_drag_brake_angle	# Target drake brake angle (degrees)
int8 flight_state			# Numerical codes to indicate flight state - see FlightState in rocket.cpp
int8 emergency_counter			# If greater than 4, emergency parachute deployment is triggered