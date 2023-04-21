README

The system, and by extension the code, consists of three distinct parts: The computer driver, the Raspberry Pi driver, 
and the Arduino driver.


The Computer

The first piece is the driver for the stereo visualization on the computer, stereovis.py. This program will begin the two 
mediapipe threads and estimate pose from them. Then using the python socket library it will send the pose information, in 
quaternions, to the Pi. This program will also show a 3d visualization of the had pose and selected IK solution though 
this IK information is not sent to the Pi.


The Raspberry Pi

The second piece is the driver for IK on the Raspberry Pi, completePipe.py. This program receives the information sent 
over ethernet, converts it back to pose, computes the IK solution, then sends it to the Arduino in the correct format.


The Arduino

The third piece is the driver for the Arduino, ArmControl452.ino. This program receives the information sent over serial 
and sets the servos to the commanded position. It performs minor adjustments so the 270 ° servos can be fully actuated 
but otherwise simply sets servos.


These three programs essentially pass information down the chain to the arm. Once they are running there is little to no 
need to supervise their operation beyond making sure the IK solution is not requesting something impossible. It is 
important to note that due to time constraints there are very few if any checks on the validity or safety of the data. 
The arm will attempt to meet whatever position it is told to with no feedback, even if this would cause the arm to 
physically damage itself. Were we to continue development a first step would likely be to add this to prevent damage.

All other files not explicitly mentioned contain function and class definitions that these three use to perform their 
functions.

Software implementation:
The software implementation for this project develops a system comprised of multiple interconnected components 
communicating with each other to provide a seamless control experience.

The first component of the software is the Stereo Computer Vision (CV) and Machine Learning (ML) system, responsible for 
capturing and processing the visual data of the user's hand. The two images are separately processed with a pre-trained 
machine learning pipeline (Mediapipe Handtrack by Google) that infers hand pose, and provides landmark coordinates for 21 
points associated with finger-joints and finger-tips. 

Then, the software leverages the landmark coordinates from both images and performs stereo processing. The stereoscopic 
projection algorithm calculates the disparity between the corresponding landmark coordinates from both cameras, and then 
uses the disparity, camera intrinsic matrix, and the camera baseline distance (distance between the two cameras) to 
compute the world-frame 3D positions (x, y, z) of the hand landmarks. 

The 3D reconstruction of the hand is then used to calculate an orthonormal frame for the hand pose, which is required for 
the input to the Inverse Kinematics algorithm. The orthonormal frame is calculated using the positions of the index 
finger and pinky, and the resulting SO(3) rotation matrix is converted to a quaternion. The calculated quaternion and 
wrist (x, y, z) position are written to the shared memory, where it is then sent over ethernet to the Raspberry Pi.

On the Raspberry Pi, the Inverse Kinematics algorithm is implemented, which calculates the joint angles to achieve the 
desired pose.

Finally, the embedded software implementation includes a control module for actuating the robotic arm based on the 
calculated joint angles. This module interfaces with the robotic arm using serial communication protocol. The joint 
angles, converted to degrees, are sent to the robotic arm via serial commands, allowing for real-time control of the 
robot's movements.