## Quick Start

Unlike most other calibration approaches, **ATOM** offers tools to address the complete calibration pipeline. These are
instructions for quick starting your robotic system calibration. If you need more details read through
the [detailed description](procedures.md) below.


### Create a calibration package 

```bash
rosrun atom_calibration create_calibration_pkg --name <your_robot_calibration>
```

### Configure a calibration package

Edit the file:

   <your_robot_calibration\>/calibration/config.yml_ with your system information.

and then run:

```bash
rosrun <your_robot_calibration> configure 
```

### Set initial estimate 

ATOM provides interactive tools based on rviz that allow the user to set the pose of the sensors to be calibrated, while receiving visual feedback.

!!! Optional

    If you consider that your initial sensor poses are already accurate, you may skip this procedure.

To use launch:

```bash
roslaunch <your_robot_calibration> set_initial_estimate.launch 
```

### Collect Data 

Collecting data produces an ATOM dataset, which is then used for calibrating the system.

```bash
roslaunch <your_robot_calibration> collect_data.launch output_folder:=~/datasets/<my_dataset> 
```

### Dataset playback

Dataset playback offers the possibility to visualize and correct the labels automatically produced during the collection stage.

!!! Optional

    If you trust that the automatic labels are correct, you may skip this procedure.


First launch the visualizer:
   
```bash
roslaunch <your_robot_calibration> dataset_playback.launch
```

and then:

```bash
rosrun atom_calibration dataset_playback -json $ATOM_DATASETS/<your_robot_calibration>/<your_dataset>/dataset.json -uic -si  -ow
```

### Calibrate sensors 

Finally run an optimization that will calibrate your sensors:

```bash
roslaunch <your_robot_calibration> dataset_playback.launch 
```

