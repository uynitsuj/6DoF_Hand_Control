README

The system, and by extent the code, consists of three distinct parts: The computer driver, the Raspberry Pi driver, and the Arduino driver.


The Computer

The first piece is the driver for the stereo visualization on the computer, stereovis.py. This program will begin the two mediapipe threads and estimate pose from them. Then using the python socket library it will send the pose information, in quaternions, to the Pi. This program will also show a 3d visualization of the had pose and selected IK solution though this IK information is not sent to the Pi.


The Raspberry Pi

The second piece is the driver for IK on the Raspberry Pi, completePipe.py. This program receives the information sent over ethernet, converts it back to pose, computes the IK solution, then sends it to the Arduino in the correct format.


The Arduino

The third piece is the driver for the Arduino, ArmControl452.ino. This program receives the information sent over serial and sets the servos to the commanded position. It performs minor adjustment so the 270° servos can be fully actuated but otherwise simply sets servos.


These three programs essential pass information down the chain to the arm. Once they are running there is little to no need to supervise their operation beyond making sure the IK solution is not requesting something impossible. It is important to note that due to time constraints there are very few if any checks on the validity or safety of the data. The arm will attempt to meet whatever position it is told to with no feedback, even if this would cause the arm to physically damage itself. Were we continue development a first step would likely be to add this to prevent damage.

All other files no explicitly mentioned contain function and class definitions that these three use to perform their functions.
