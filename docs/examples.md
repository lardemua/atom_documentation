## Calibration Examples

ATOM provides extensive visualization possibilities while running the calibration optimization procedure. To visualize in ROS Rviz use the -rv flag.

<!-- [![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/1NOEBKDMIpk/0.jpg)](https://www.youtube.com/watch?v=1NOEBKDMIpk) -->


So far, we have used **ATOM** to successfully calibrate several robotic platforms. Here are some examples:

### Atlascar2
 
[Atlascar2](https://github.com/lardemua/atlascar2) is an intelligent vehicle containing several cameras and 2D Lidars. 


<p align="center">
  <img width="90%" src="/img/atlas_nocircles.png">
</p>
<p align = "center">
A photograph of the AtlasCar2.
</p>

<p align="center">
  <img width="80%" src="/img/atlascar2_with_markers_first_guess.png">
</p>
<p align = "center">
A 3D Model of the AtlasCar2.
</p>


### IrisUA - ur10e 
The [IrisUA - ur10e](https://github.com/iris-ua/iris_ur10e_calibration) includes several variants of the hand-eye calibration problem.


### AgrobV2 
 [AgrobV2](https://github.com/aaguiar96/agrob) is a mobile robot with a stereo camera and a 3D Lidar designed for agriculture robotics.

![AgrobV2](/img/agrob_calibration.gif){: style="width:100em"}

*Calibration of AgrobV2.*


### LARCC

**L**aboratory of **A**utomation and **R**obotics **C**ollaborative **C**ell (LARCC) is included in a research project focusing of collaborative robotic industrial cells. The goal is to monitor in detail the volume of the cell in order to ensure safe collaboration between human operators and robots. For this, several sensors of different modalities are positioned everywhere in the cell, which makes the calibration of this robotic system a challenging task.



![AgrobV2](/img/larcc_fovs.png){: style="width:100em"}
*Sensor fields of view in LARCC.*


![AgrobV2](/img/larcc_calibration.png){: style="width:100em"}
*Calibration of LARCC.*



### MMTBot
 [MMTBot](https://github.com/miguelriemoliveira/mmtbot) is a simulated robotic system containing a manipulator, two rgb cameras and one 3D lidar, with the goal of reserching how ATOM can calibration hand-eye systems.

