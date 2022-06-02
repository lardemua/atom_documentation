<!-- <img align="left" width="375" height="215" src="https://github.com/lardemua/atom/blob/noetic-devel/docs/atom_logo.png?raw=true/375/215"> ATOM Calibration
 -->

![AtomLogo](/img/atom_logo.png){: style="width:25em"}


## ATOM 

A Calibration Framework using **A**tomic **T**ransformations **O**ptimization **M**ethod.

**ATOM** is a set of calibration tools for multi-sensor, multi-modal, robotic systems, based on the optimization of
atomic transformations as provided by a ros based robot description.
Moreover, **ATOM** provides several scripts to facilitate all the steps of a calibration procedure.

If this work is helpful for you please cite [our papers](publications.md).


<!-- ## Table of Contents -->

 <!-- * [How to Use - Quick Start](#how-to-use---quick-start)
  * [Examples](#examples)
    + [Atlascar2](#atlascar2)
    + [IrisUA - ur10e](#irisua---ur10e)
    + [AgrobV2](#agrobv2)
    + [LARCC](#larcc)
    + [MMTBot](#mmtbot)
  * [System calibration - Detailed Description](#system-calibration---detailed-description)
    + [Setup you environment](#setup-you-environment)
    + [Creating a calibration package](#creating-a-calibration-package)
    + [Configuring a calibration package](#configuring-a-calibration-package)
    + [Set initial estimate](#set-initial-estimate)
    + [Collect data](#collect-data)
    + [Calibrate sensors](#calibrate-sensors)
        * [Advanced usage - running calibration script in separate terminal](#advanced-usage---running-calibration-script-in-separate-terminal)
        * [Advanced usage - two stage calibration for robotic systems with an anchored sensor](#advanced-usage---two-stage-calibration-for-robotic-systems-with-an-anchored-sensor)
  * [Evaluating your calibration](#evaluating-your-calibration)
      - [Annotation](#annotation)
      - [Camera-to-Camera evaluation](#camera-to-camera-evaluation)
      - [LiDAR-to-Depth-Camera evaluation](#lidar-to-depth-camera-evaluation)
      - [Camera-to-Depth-Camera evaluation](#camera-to-depth-camera-evaluation)
      - [LiDAR-to-LiDAR evaluation](#lidar-to-lidar-evaluation)
      - [LiDAR-to-Camera evaluation](#lidar-to-camera-evaluation)
      - [Point cloud image projection](#point-cloud-image-projection)
  * [Installation](#installation)
  * [Contributors](#contributors)
  * [Maintainers](#maintainers)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small> -->


## How to Use - Quick Start

[Quick Start](quick_start.md)

## Examples

ATOM provides extensive visualization possibilities while running the calibration optimization procedure. To visualize in ROS Rviz use the -rv flag.

<!-- [![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/1NOEBKDMIpk/0.jpg)](https://www.youtube.com/watch?v=1NOEBKDMIpk) -->


So far, we have used **ATOM** to successfully calibrate several robotic platforms. Here are some examples:

### Atlascar2
 

### IrisUA - ur10e 

### AgrobV2 


### LARCC

**L**aboratory of **A**utomation and **R**obotics **C**ollaborative **C**ell (LARCC) is included in a research project focusing of collaborative robotic industrial cells. The goal is to monitor in detail the volume of the cell in order to ensure safe collaboration between human operators and robots. For this, several sensors of different modalities are positioned everywhere in the cell, which makes the calibration of this robotic system a challenging task.


| <img align="center" src="docs/larcc_fovs.png" width="600"/>
|:--:| 
| Sensor fields of view in LARCC.|

| <img align="center" src="docs/larcc_calibration.png" width="800"/>
|:--:| 
| Calibration of LARCC.|



### MMTBot





## Installation

    sudo pip3 install -r requirements.txt

## Contributors

* Miguel Riem Oliveira - University of Aveiro
* Afonso Castro - University of Aveiro
* Eurico Pedrosa - University of Aveiro
* Tiago Madeira - University of Aveiro
* André Aguiar - INESC TEC
* Daniela Rato - University of Aveiro

## Maintainers

* Miguel Riem Oliveira - University of Aveiro
* Daniela Rato - University of Aveiro
* Manuel Gomes - University of Aveiro


