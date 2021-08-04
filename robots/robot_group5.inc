define rob5_laser ranger
(
	sensor(
		range_max 1.
		fov 360
		samples 360
	)
)

define rob5 position
(
	# basic attributes of the robot
	size [1. 1. 1.]
	origin [0. 0. 0. 0.]
	color "honeydew2"

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
		point[1] [ -0.086 0.208 ]
		point[2] [ -0.076 0.198 ]
		point[3] [ 0.215 0.000 ]
		z [0.5 1.]
	)

	block
	(
		points 4
		point[0] [ 0.225 0.000 ]
		point[1] [ -0.086 -0.208 ]
		point[2] [ -0.076 -0.198 ]
		point[3] [ 0.215 0.000 ]
		z [0.5 1.]
	)

	block
	(
		points 4
		point[0] [ -0.086 -0.208 ]
		point[1] [ -0.086 0.208 ]
		point[2] [ -0.076 0.198 ]
		point[3] [ -0.076 -0.198 ]
		z [0.5 1.]
	)

	# GUI properties
	gui_nose 0
	gui_grid 0
	gui_outline 1
	gui_move 1 (1 if the model has no parents);

	# locallization properties
	localization "odom"
	odom_error [ 0.01 0.01 0.0 0.05 ]

	rob5_laser(pose [ 0.225 0.000 0.])
)
