## DIY Robot Arm V2
This repository is for archiving a prior 6 DOF robot project that is no longer maintained.
I am now working on the next V3 version instead. 

### Contents
| Folder  | Contents|
| ------------- | ------------- |
| [Adaptor Board](https://github.com/JasonHEngineering/DIY-Robot-Arm-V2/tree/main/Adaptor%20Board%20(schema%2C%20gerbers%2C%20kicad%20project%20folder)) | KiCad Project files. Gerber files for PCB. But no SMD assembly because for this variant of adaptor board, I did the soldering myself using both soldering iron and hot air solder. |
| [Arduino Nano](https://github.com/JasonHEngineering/DIY-Robot-Arm-V2/tree/main/Arduino%20Nano%20(.ino%20file)) | Load this file into Arduino Nano 33 BLE/IoT device. This is to be mounted into adaptor board above. |
| [CAD files](https://github.com/JasonHEngineering/DIY-Robot-Arm-V2/tree/main/CAD%20files%20(2d%2C%20step)) | Step files and 2d drawings. 2d drawings are not fully detailed; mainly the threads and tight tolerances are detailed. |
| [Raspberry Pi](https://github.com/JasonHEngineering/DIY-Robot-Arm-V2/tree/main/Raspberry%20Pi%20(plotly%20dash%20host)) | Python script and dependencies to be run on raspberry Pi 3/4 |

### Mechanical Structure
Machined CNC parts placement in assembly as shown below. 
> [!NOTE]
> In such a setup, micro limit switches can only zero the respective axis angles approximately. Next version shall use absolute encoders.    
![](https://github.com/JasonHEngineering/DIY-Robot-Arm-V2/blob/main/Images/structure.JPG?raw=true)

Below illustration of user interface dealing with forward kinematics values:  
![Images/illustration_1.JPG](https://github.com/JasonHEngineering/DIY-Robot-Arm-V2/blob/main/Images/illustration_1.JPG?raw=true)

### Inverse Kinematics Solver
[Python solver](https://github.com/JasonHEngineering/Inverse-Kinematics)
1. Inverse Kinematics (CCD) – Cyclic Coordinate Descent in 3d space
2. Inverse Kinematics (vector formula Jacobian) - Euler
3. Inverse Kinematics (Numerical Jacobian) - Euler
4. Inverse Kinematics (Numerical Jacobian) - Quaternion

### Quick Illustration at YouTube
[![DIY Robot Arm V2](https://img.youtube.com/vi/ZaQQ6-Mw-B4/0.jpg)](https://www.youtube.com/watch?v=ZaQQ6-Mw-B4)

### Additional Info at WordPress
For more data/info, please refer to this [WordPress page](https://jashuang1983.wordpress.com/robotics-6-dof-arm-v2/).
