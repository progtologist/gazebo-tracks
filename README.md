Gazebo-Tracks
=============

A script that can create a track driven sdf model for gazebo.
The tracks are created by adding many small pads in the perimeter with a rotational joint in between.
To alter the dimensions of the robot edit the file settings.config
minimum_pads is essential for the tuning of the model to determine collision problems with the drums. If the model explodes due to collision, try increasing the minimum_pads.
The more the minimum_pads, the better the approximation of a real track, however, the computational cost increases significantly.

How to use:
-----------

You need to edit the variables set in the file "settings.config"

	track_separation = 0.340
	drum_radius = 0.07
	drum_width = 0.1
	drum_separation = 0.56
	pad_width = 0.1
	pad_thickness = 0.001
	minimum_pads = 30
	output = 'track_creator/model.sdf'

track_separation sets the distance between the left and right tracks
drum_separation sets the wheelbase of the robot, or the distance of the front and rear drum in the tracks
drum_radius sets the radius of the drums of the tracks
drum_width sets the width of the cylinder and therefore the width of each track
pad_width sets the width of the pads, should be set equal to the drum_width
pad_thickness sets the thickness of the pads, usually a small value would suffice
minimum_pads sets the minimum number of pads acceptable for each of the tracks for the approximation
output sets the filename of the resulted sdf model

To run the script:
------------------

Simply navigate to the folder and execute the script track_creator.py
In linux simply type:

	./track_creator.py

The default model provided creates the robot shown in track_creator/model.png