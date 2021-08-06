define rob5_laser ranger
(
	sensor(
		range [0.0 3.0]
		fov 360.
		samples 360
	)
	pose [ 0.215 0.000 0. 0. ]
	size [ 0.01 0.01 0.01 ]
)

define rob5 position
(
	# basic attributes of the robot
	size [1. 1. 0.5]
	origin [0. 0. 0. 0.]
	color "honeydew2"
	drive "diff"

	# shape of the robot
	block
	(
		points 16
		point[0] [ 0.225 0.000 ]
		point[1] [ 0.208 0.086 ]
		point[2] [ 0.159 0.159 ]
		point[3] [ 0.086 0.208 ]
		point[4] [ 0.000 0.225 ]
		point[5] [ -0.086 0.208 ]
		point[6] [ -0.159 0.159 ]
		point[7] [ -0.208 0.086 ]
		point[8] [ -0.225 0.000 ]
		point[9] [ -0.208 -0.086 ]
		point[10] [ -0.159 -0.159 ]
		point[11] [ -0.086 -0.208 ]
		point[12] [ -0.000 -0.225 ]
		point[13] [ 0.086 -0.208 ]
		point[14] [ 0.159 -0.159 ]
		point[15] [ 0.208 -0.086 ]
		z [0 0.5]
	)

	block
	(
		points 4
		point[0] [ 0.225 0.000 ]
		point[1] [ -0.159 0.159 ]
		point[2] [ -0.139 0.149 ]
		point[3] [ 0.215 0.000 ]
		z [0.5 1.]
	)

	block
	(
		points 4
		point[0] [ 0.225 0.000 ]
		point[1] [ -0.159 -0.159 ]
		point[2] [ -0.139 -0.149 ]
		point[3] [ 0.215 0.000 ]
		z [0.5 1.]
	)

	block
	(
		points 4
		point[0] [ -0.159 -0.159 ]
		point[1] [ -0.139 -0.149 ]
		point[2] [ -0.139 0.149 ]
		point[3] [ -0.159 0.159 ]
		z [0.5 1.]
	)

	# GUI properties
	gui_nose 0
	gui_grid 0
	gui_outline 1
	gui_move 1

	# locallization properties
	localization "odom"
	odom_error [ 0.01 0.01 0.0 0.05 ]

	rob5_laser()
)
