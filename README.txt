-This is a rotary inverted pendulum simulated in vrep and is currently held semi-stable by a PI controller written in python.
-Communication occurs through vreps python API
-Joint two (pendulum base) is treated as a rotary encoder and the controller can only see its position at any given time.
-Joint one is treated as a DC motor, in which the controller sets target velocities in order to achieve desired results.

There are two folders within this folder:
- “Python Code and Vrep Files” contains all files needed to recreate the simulation
	- Note: Depending on the host computer’s OS, the .dll file may need to be swapped out. Refer to the Vrep-Python API documentation for more information.
- “Video” contains a video of the project running.
