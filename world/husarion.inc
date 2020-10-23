include "rplidar_a2m7.inc"

define husarion position
(
color "red"          # Default color.
drive "diff"         # Differential steering model.
gui_nose 1           # Draw a nose on the robot so we can see which way it points
obstacle_return 1    # Can hit things.
laser_return 1	            	# reflects sonar beams


# alternative odometric localization with simple error model
localization "gps"

# four DOF kinematics limits
velocity_bounds     [-1.0 1.0 0 0 0 0 -60.0 60.0 ]
acceleration_bounds [-0.5 0.5 0 0 0 0 -60.0 60.0 ]

# Actual size
size [0.2 0.235 0.22]

# The pioneer's center of rotation is offset from its center of area
	origin [0 0 0 0]

# Estimated mass in KG
mass 10.0


rplidar( pose [0.0 0 -0.2 0 ])
)

