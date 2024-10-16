## Parametric Surface Reconsctruction from Point Cloud using Creo Toolkit (Creo Parametric)
Supporting code for the paper:<br><br>
![image](https://github.com/user-attachments/assets/bba06439-373c-426e-9125-859252d27ccb)<br><br>
This is a self contained Visual Studio Project that builds the plugin using the libraries needed. The only thing needed is to set in the environment variables the Creo Toolkit Path in the variable *PROTOOL_SRC*. Usually this path is in the directory C:\Program Files\PTC\. For example,  see the image below:<br><br>
![Environmental Variable](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/2.jpg?raw=true)<br><br>
After setting this environment variable the code should be able to compile and build in Visual Studio by selecting the ReleaseOpenMP option in the build settings. <br>
The plugin is built in the directory **CreoUI** so the directory should look like in the image below. The DLL built shoud be **CreoSurfaceReconstructionFromPC.dll**:<br><br>
![DLL](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/3.jpg?raw=true)<br><br>
## Most proper way to proceed
The protk.dat file in the folder CreoUI contains all the information for the plugin to load.<br>
The follpwing steps need to be followed for the plugin to exectute:<br>

 1. Execute Creo Parametric
 2. Go to File->Options->Options->Environment->Working directory->Browse... and open the directory where CreoUI is, see image below. Make sure the protk.dat is in this directory.
 3. Press ok.
 4. Start a part session

![Selecting Working Directory](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/8.jpg?raw=true)
<br><br>
After following the above steps you should be able to run the plugin in Creo Parametric. To do so open a part session in it and go to the **Tools** tab, click on **Auxiliary Applications** and select the protk.dat file in the folder the plugin was created like in the image below:<br><br>
![enter image description here](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/6.jpg?raw=true)
<br><br>
After you open the plugin you need to press start and provided you followed all steps correctly the plugin should be in the state running. Then on the right a button tools should appear. By pressing on it you can run the application like in the image below:<br><br>
![enter image description here](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/7.jpg?raw=true)<br><br>
Please watch the video below on how to use the plugin:<br><br>
[![Watch the video](https://github.com/agalex1974/CreoSurfaceReconstructionFromPC/blob/main/ReadMeImages/sor.jpg?raw=true)](https://youtu.be/2iVubCjAmTw)


