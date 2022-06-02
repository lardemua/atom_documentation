## Quick Start

Unlike most other calibration approaches, **ATOM** offers tools to address the complete calibration pipeline. These are
instructions for quick starting your robotic system calibration. If you need more details read through
the [detailed description](docs/detailed_description.md) below.

Also, you can take a look at the
ATOM [youtube playlist](https://www.youtube.com/watch?v=BYs1-H9vh0s&list=PLQN09mzV5mbI4h5IQt3Eu9kugSk-08mnY).

### Create a calibration package 


```bash
rosrun atom_calibration create_calibration_pkg --name <your_robot_calibration>
```

### Configure a calibration package

Edit the file:

   _<your_robot_calibration>/calibration/config.yml_ with your system information.

```bash
rosrun <your_robot_calibration> configure 
```

### Set initial estimate 

 [_optional_] - deployment of interactive tools based on rviz that allow the user to set the

   pose of the sensors to be calibrated, while receiving visual feedback;

```bash
roslaunch <your_robot_calibration> set_initial_estimate.launch 
```

### Collect Data 

- Extraction of snapshots of data (a.k.a., collections) which constitute an ATOM dataset:

```bash
roslaunch <your_robot_calibration> collect_data.launch output_folder:=~/datasets/<my_dataset> 
```

### Dataset playback

Manual Annotation** [_optional_] - it is possible to visualize the labels automatically produced during the collection stage and correct them mannually:
   
```bash
roslaunch <your_robot_calibration> dataset_playback.launch
```
and then:
```bash
rosrun atom_calibration dataset_playback -json $ATOM_DATASETS/<your_robot_calibration>/<your_dataset>/dataset.json -uic -si  -ow
```

### Calibrate sensors 

- finally run an optimization that will calibrate your sensors:

```bash
roslaunch <your_robot_calibration> dataset_playback.launch 
```

