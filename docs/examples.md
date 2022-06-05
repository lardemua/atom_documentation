## Calibration Examples

ATOM provides extensive visualization possibilities while running the calibration optimization procedure. To visualize in ROS Rviz use the -rv flag.

<!-- [![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/1NOEBKDMIpk/0.jpg)](https://www.youtube.com/watch?v=1NOEBKDMIpk) -->


So far, we have used **ATOM** to successfully calibrate several robotic platforms. Here are some examples:

### MMTBot
 [MMTBot](https://github.com/miguelriemoliveira/mmtbot) is a simulated robotic system containing a manipulator, two rgb cameras and one 3D lidar, with the goal of reserching how ATOM can calibration hand-eye systems.

<p align="center">
  <img width="100%" src="/img/robotic_systems/mmtbot_rviz4.png">
</p>
<p align = "center">
A 3D Model of the MMTBot.
</p>

### Atlascar2
 
 The [Atlascar2](https://github.com/lardemua/atlascar2) is an intelligent vehicle containing several cameras and 2D Lidars. This was the first platform we have calibrated using ATOM. The repositories containing the atlascar packages are here:

<p align="center">
<a href="https://github.com/lardemua/atlascar2">https://github.com/lardemua/atlascar2</a>
</p>


<p align="center">
  <img width="90%" src="/img/robotic_systems/atlas_nocircles.png">
</p>
<p align = "center">
A photograph of the AtlasCar2.
</p>

<p align="center">
  <img width="80%" src="/img/robotic_systems/atlascar2_with_markers_first_guess.png">
</p>
<p align = "center">
A 3D Model of the AtlasCar2.
</p>

Further details on this system can be read in the [papers published in ROBOT 2019 and RAS 2020](publications.md).


### IrisUA - ur10e 
The [IrisUA - ur10e](https://github.com/iris-ua/iris_ur10e_calibration) includes several variants of the hand-eye calibration problem. The repositories containing the calibration ros package for this system are here: 

<p align="center">
<a href="https://github.com/iris-ua/iris_ur10e_calibration">https://github.com/iris-ua/iris_ur10e_calibration</a>
</p>


<p align="center">
  <img width="70%" src="/img/robotic_systems/eye-to-base-real_alpha.png">
</p>
<p align = "center">
A photograph of the IRIS UA UR10e.
</p>

<p align="center">
  <img width="60%" src="/img/robotic_systems/set_initial_estimate_joint_hand_base_fixed.png">
</p>
<p align = "center">
A 3D Model of the IRIS UA UR10e.
</p>

Further details on this system can be read in the [paper published in T-Ro 2021](publications.md).


### AgrobV2 
 [AgrobV2](https://github.com/aaguiar96/agrob) is a mobile robot with a stereo camera and a 3D Lidar designed for agriculture robotics.


<p align="center">
  <img width="50%" src="/img/robotic_systems/agrobv16.png">
</p>
<p align = "center">
A photograph of the AgrobV2.
</p>


<p align="center">
  <img width="50%" src="/img/robotic_systems/agrob_rviz.png">
</p>
<p align = "center">
A 3D Model of the AgrobV2.
</p>


Further details on this system can be read in the [paper published in ESWA 2021](publications.md).

### LARCC

**L**aboratory of **A**utomation and **R**obotics **C**ollaborative **C**ell (LARCC) is included in a research project focusing of collaborative robotic industrial cells. The goal is to monitor in detail the volume of the cell in order to ensure safe collaboration between human operators and robots. For this, several sensors of different modalities are positioned everywhere in the cell, which makes the calibration of this robotic system a challenging task.


<p align="center">
  <img width="100%" src="/img/robotic_systems/larcc_real.jpeg">
</p>
<p align = "center">
A photograph of LARCC.
</p>


<p align="center">
  <img width="100%" src="/img/robotic_systems/larcc_sim.png">
</p>
<p align = "center">
A 3D Model of the LARCC.
</p>



![AgrobV2](/img/larcc_fovs.png){: style="width:100em"}
*Sensor fields of view in LARCC.*


![AgrobV2](/img/larcc_calibration.png){: style="width:100em"}
*Calibration of LARCC.*



