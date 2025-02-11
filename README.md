The main code for robotic arm connects intel d435i depth sensor via usb and starts displaying a live footage on sscreen you can select the 4 points of workbench to caliberate
then the line to weld must be drawn inside the workbench which will convert pixels into real world coordinates using homography and feed the points array to inverse kinematics function
the inverse kinematics function takes input as x,y,z coordinates and convert it3 angles of motors which will make the tip of arm in (x,y,z) point the angles are then fed to movemotors function
the movemotors finds the angle to move from current arm position and input and sents data to arduino via USB
the arduino recieves and parses data and move 3 motors as per revieved data through a motor controller
the workbench dimensions, motor driver parameters, gearbox ration is to be edited as per use
