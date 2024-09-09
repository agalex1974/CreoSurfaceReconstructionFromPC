## Parametric Surface Reconsctruction from Point Cloud using Creo Toolkit (Creo Parametric)
Work in Progress for paper submission: **Feature Reconstruction Methods in Point Clouds with Applications in Creo Parametric**.<br>
***All rights reserved to Alexander Agathos, Sofia Kyratzi and Philip Azariadis***.<br>
This is a self contained Visual Studio Project that builds the plugin using the libraries needed. The only thing needed is to set in the environment variables the Creo Toolkit Path in the variable *PROTOOL_SRC*. Usually this path is in the directory C:\Program Files\PTC\. For example,  see the image below:<br>
![Environmental Variable](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/2.jpg?raw=true)<br> 
After setting this environment variable the code should be able to compile and build in Visual Studio by selecting the ReleaseOpenMP option in the build settings. <br>
The plugin is built in the directory **CreoUI** so the directory should look like in the image below. The DLL built shoud be **CreoSurfaceReconstructionFromPC.dll**:<br>
## Most proper way to proceed
The protk.dat file in the folder CreoUI contains the information for the plugin to load.<br>
The follpwing steps need to be followed for the plugin to exectute:<br>

 1. Execute Creo Parametric
 2. Go to File->Options->Options->Environment->Working directory->Browse... and open the directory where CreoUI is, see image below. Make sure the protk.dat is in this directory.
 3. Press ok.

![Selecting Working Directory](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/8.jpg?raw=true)
<br>
After following the above steps you should be able to run the plugin in Creo Parametric. To do so open a part session in it and go to the **Tools** tab click on **Auxiliary Applications** and select the protk.dat file in the folder the plugin was created like in the image below:<br>
![enter image description here](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/6.jpg?raw=true)
<br>
After you open the plugin you need to press start and provided you followed all steps correctly the plugin should be in the state running. Then on the right a button tools should appear. By pressing on it you can run the application like in the image below:<br>
![enter image description here](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/7.jpg?raw=true)
