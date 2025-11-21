# non-parametric-jacobian

## Project Statement
Soft robots have the potential to navigate tortuous pathways such as those found in the human anatomy owing to their compliant nature and low risk of injury to soft tissues. But, their control is challenging due to the lack of a reliable closed-form analytical model. 

## Relevant Files Locations
There could be more files you need, I wanted to point out where the most useful things are located. 

```
├── final presentation.pptx    # Final presentation that was given as part of a class project, not the most detailed
├── nonparamjacobian.pdf    # The initial paper draft, contains more technical information
├── Code                             # My poorly managed code repository
│   ├── onlineJacobianCode   # The control code is located here
│   │   └── pythonJacobianCode_v5.py # Should be the only python file you need, more information below
│   │   └── cameraCalibrationData.json  # Calibration data for camera, created in the code _v5.py file if it doesn't exist. I believe is to convert from CV to numpy. 
│   │   └── control_path_data.json  # This is the output of the program which gives the xy cordinates in pixel space for both the actual trajectory of the robot and the desired trajectory of the robot when moving to a new point in space
│   │   └── initialJacobian.json  # Once an initial space expolration is completed this is the Jacobian that the robot will start with when at the default position. 
├── Schematics and Drawings  # Contains schematics and drawings from the  project
│   ├── sw
│   │   └── EE Robot.png  # An incredibly simple schematic of the robot
```
## nonparamjacobian.pdf is the most useful documentation
It contains...
- Simple schematics
- Outline of the process of space exploration -> control
- Actual math behind this 
- The results we will be trying to replicate

## pythonJacobianCode_v5.py Information
**Perhaps the most important piece of information, this was created in Python 2.7 and will not iniately run on Python 3 if you try to test it as is.**

The overall gist is that the program uses the CV aruco markers on the end of the soft end effector to track the 2D position of the end effector in pixel space. The program will be able to create the needed input/output files (I believe) which are mostly json files for storing and retrieving information (json files explained in file tree above). The program takes user input to make chooses on what the program should do (calibration vs running) and will pop up a window showing the camera view when running. 

The actual process is that an initial Jacobian is created by calibrating the robot by doing a series of known commands resulting in a space expolration. Once that is complete the robot will then use that Jacobian which is updated as it moves to continue controlling the movement. We only look at 2D space as the desire is to use this for navigating paths where you are not controlling Z but only what direction will the robot go once the operator moves Z manually. 

The motors communicate over serial to python. 

**Whenever the program is run the control_path_data.json is overwritten. I never had a chance to put in file. **

The code is decently well commented, I can provide further clarity as needed. 






