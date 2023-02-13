# Safety Constrained Shared Control (SCSC)
This repository is for running the SCSC algorithm in real-time using a commercial joystick controller to control the ball-in-bowl system.

## Downloading Eigen
This repo uses the Eigen library available at `https://eigen.tuxfamily.org/index.php?title=Main_Page`. Version 3.4.0 is recommended.
Take note of where you save these files and add the path (e.g., `C:\...\eigen-3.4.0\eigen-3.4.0\Eigen`) in steps 3d and 3e.

## Visual Studio Setup
This repository utilizes Microsoft Visual Studio and OpenGL. To set up a new project in VS follow these steps:

0. Copy `glut32.dll` and `glfw.dll` into `C:\Windows\SysWOW64` (You should only have to do this once per computer.)
1. Create an empty project in VS
2. Add main file to the Source folder
3. In Configuration Properties:  
    a. Set target to console: `Linker>System>SubSystem>Console (/SUBSYSTEM:CONSOLE)`  
    b. Turn incremental linking off: `Linker>General>Enable Incremental Linking>No (/INCREMENTAL:NO)`  
    c. Add library files: `Linker>Input>Additional Dependencies>Edit` add `glut32.lib` and `glfw3.lib`   
    d. Add directory containing library files to search path: `VC++ Directories>Library Directories`  (e.g., `C:\...\SafetyConstrainedSharedControl\Game Code`)  
    e. Add directory containing header files: `VC++ Directories>Include Directories`  (e.g., `C:\...\SafetyConstrainedSharedControl\Game Code`)  

### File Paths
If you are using this repo on a different computer, the file path for the table texture will need to be updated at the top of `texture.h`.  

### Troubleshooting  
If the contents of `Game Code/stb` are missing when cloning this repo, try `git submodule update --init --recursive --remote`.  
