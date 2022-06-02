## System calibration - Detailed Description

To calibrate your robot you must define your robotic system, (e.g. <your_robot>). You should also have a **system
description** in the form of an urdf or a xacro file(s). This is normally stored in a ros package named **<your_robot>_
description**.

Finally, **ATOM** requires a bagfile with a recording of the data from the sensors you wish to calibrate.
Transformations in the bagfile (i.e. topics /tf and /tf_static) will be ignored, so that they do not collide with the
ones being published by the _robot_state_publisher_. Thus, if your robotic system contains moving parts, the bagfile
should also record the _sensor_msgs/JointState_ message.

It is also possible to record compressed images, since **ATOM** can decompress them while playing back the bagfile. Here
is
a [launch file example](https://github.com/lardemua/atlascar2/blob/master/atlascar2_bringup/launch/record_sensor_data.launch)
which records compressed images.

### Setup you environment

We often use two enviroment variables to allow for easy cross machine access to bagfiles and datasets. If you want to
use these (it is optional) you can also add these lines to your _.bashrc_:

```bash
export ROS_BAGS="$HOME/bagfiles"
export ATOM_DATASETS="$HOME/datasets"
```

and then you can refer to these environment variables when providing paths to atom scripts, e.g.:

```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=$ATOM_DATASETS/<my_dataset>/dataset.json
```

and you can also refer to them inside
the [calibration configuration file](https://github.com/lardemua/atlascar2/blob/0c065508f325fb57e0439c1ba2e00f9468cd73e7/atlascar2_calibration/calibration/config.yml#L14)

### Creating a calibration package

To start you should create a calibration ros package specific for your robot. **ATOM** provides a script for this:

```bash
rosrun atom_calibration create_calibration_pkg --name <your_robot_calibration>
```

This will create the ros package <your_robot_calibration> in the current folder, but you can also specify the folder,
e.g.:

```bash
rosrun atom_calibration create_calibration_pkg --name ~/my/path/<your_robot_calibration>
```

### Configuring a calibration package

Once your calibration package is created you will have to configure the calibration procedure by editing the
_<your_robot_calibration>/calibration/config.yml_ file with your system information.

Here are examples of calibration **config.yml** files for
an [autonomous vehicle](https://github.com/lardemua/atlascar2/blob/master/atlascar2_calibration/calibration/config.yml)
and for
a [simulated hand eye system](https://github.com/miguelriemoliveira/mmtbot/blob/main/mmtbot_calibration/calibration/config.yml)

After filling the config.yml file, you can run the package configuration:

```bash
rosrun <your_robot_calibration> configure 
```

This will go through a series of varifications, and create a set of files for launching the system, configuring rviz,
etc.

It is also possible to configure your calibration package with a different configuration file, in the case you have
multiple configurations with multiple config.yml files. To do this, you can use:

```bash
rosrun <your_robot_calibration> configure -c new_config_file.yml
```

If you want to use other arguments of the calibration package configuration you may run:

```bash
rosrun atom_calibration configure_calibration_package  --name <your_robot_calibration> <other options>
```

```bash
usage: configure_calibration_pkg [-h] -n NAME [-utf] [-cfg CONFIG_FILE]

-h, --help            show this help message and exit
-n NAME, --name NAME  package name
-utf, --use_tfs       Use transformations in the bag file instead of generating new tfs from the xacro,
                      joint_state_msgs and robot state publisher.
-cfg CONFIG_FILE, --config_file CONFIG_FILE
                      Specify if you want to configure the calibration package with a specific configutation file.
                      If this flag is not given, the standard config.yml ill be used.
```

### Set initial estimate

Iterative optimization methods are often sensitive to the initial parameter configuration. Here, the optimization
parameters represent the poses of each sensor. **ATOM** provides an interactive framework based on rviz which allows the
user to set the pose of the sensors while having immediate visual feedback.

To set an initial estimate run:

```bash
roslaunch <your_robot_calibration> set_initial_estimate.launch 
```

Here are a couple of examples:

[Atlascar2](https://github.com/lardemua/atlascar2)  | [AgrobV2](https://github.com/aaguiar96/agrob) | [UR10e eye in hand](https://github.com/iris-ua/iris_ur10e_calibration)
------------- | ------------- | -------------
<img align="center" src="docs/set_initial_estimate_atlascar2.gif" width="450"/> | <img align="center" src="docs/agrob_initial_estimate.gif" width="450"/> | <img align="center" src="docs/ur10e_eye_in_hand_set_initial_estimate.gif" width="450"/>


### Collect data 

To run a system calibration, one requires sensor data collected at different time instants. We refer to these as **data collections** or simply **collections**. To collect data, the user should launch:

```bash
roslaunch <your_robot_calibration> collect_data.launch  output_folder:=<your_dataset_folder>
```

Depending on the size and number of topics in the bag file, it may be necessary (it often is) to reduce the playback
rate of the bag file.

```bash
roslaunch <your_robot_calibration> collect_data.launch  output_folder:=<your_dataset_folder> bag_rate:=<playback_rate>
```

You can use a couple of launch file arguments to configure the calibration procedure, namely

* **overwrite** [false] - if the dataset folder is the same as one previously recorded, it overwrites the previous
  dataset
* **ssl** [false] - **S**kip **S**ensor **L**abelling: A string to be evaluated into a lambda function that receives a
  sensor name as input and returns True or False to indicate if that sensor should be labelled. An example:
   ```
    roslaunch <your_robot_calibration> collect_data.launch 
      output_folder:=$ATOM_DATASETS/<my_dataset>/
      ssl:='lambda name: name in ["lidar_1", "lidar_2", "lidar_3"]'


Here are some examples of the system collecting data:

[Atlascar2](https://github.com/lardemua/atlascar2)  | [AgrobV2](https://github.com/aaguiar96/agrob) | [UR10e eye to_base](https://github.com/iris-ua/iris_ur10e_calibration)
------------- | ------------- | -------------
<img align="center" src="docs/collect_data_atlascar2.gif" width="450"/>  | <img align="center" src="docs/agrob_data_collection.gif" width="450"/> | <img align="center" src="docs/ur10e_eye_to_base_collect_data.gif" width="450"/>

A dataset is a folder which contains a set of collections. There, a _dataset.json_ file stores all the information
required for the calibration. There are also in the folder images and point clouds associated with each collection.

<img align="center" src="docs/viewing_data_collected_json.gif" width="600"/> 

### Calibrate sensors

Finally, a system calibration is called through:

```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=~/datasets/<my_dataset>/dataset.json
```

You can use a couple of launch file arguments to configure the calibration procedure, namely

* **single_pattern** [false] -
* **use_incomplete_collections** [false] - Remove any collection which does not have a detection for all sensors.
* **ssf** [false] - **S**ensor **S**election **F**unction: A string to be evaluated into a lambda function that receives
  a sensor name as input and returns True or False to indicate if the sensor should be loaded (and used in the
  optimization). An example:
    ```
    roslaunch <your_robot_calibration> calibrate.launch 
      dataset_file:=$ATOM_DATASETS/<my_dataset>/dataset.json  
      ssf:='lambda name: name in ["camera1, "lidar2"]'
    ```
* **csf** [false] - **C**ollection **S**election **F**unction: A string to be evaluated into a lambda function that
  receives a collection name as input and returns True or False to indicate if that collection should be loaded (and
  used in the optimization). An example:
   ```
    roslaunch <your_robot_calibration> calibrate.launch 
      dataset_file:=$ATOM_DATASETS/<my_dataset>/dataset.json  
      csf:='lambda name: int(name) < 7'
    ```

##### Advanced usage - running calibration script in separate terminal

Alternatively, for debugging the calibrate script it is better not to have it executed with a bunch of other scripts
which is what happens when you call the launch file. You can run everything with the launch excluding without the
calibrate script

```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=~/datasets/<my_dataset>/dataset.json run_calibration:=false 
```

and then launch the script in standalone mode

```bash
rosrun atom_calibration calibrate -json dataset_file:=~/datasets/<my_dataset>/dataset.json 
```

There are several additional command line arguments to use with the **calibrate** script, run calibrate --help to get the complete list:

```bash
usage: calibrate [-h] [-sv SKIP_VERTICES] [-z Z_INCONSISTENCY_THRESHOLD]
                 [-vpv] [-vo] -json JSON_FILE [-v] [-rv] [-si] [-oi] [-pof]
                 [-sr SAMPLE_RESIDUALS] [-ss SAMPLE_SEED] [-od] [-fec] [-uic]
                 [-rpd] [-ssf SENSOR_SELECTION_FUNCTION]
                 [-csf COLLECTION_SELECTION_FUNCTION]

optional arguments:
  -h, --help            show this help message and exit
  -json JSON_FILE, --json_file JSON_FILE
                        Json file containing input dataset.
  -vo, --view_optimization
                        Displays generic total error and residuals graphs.
  -v, --verbose         Be verbose
  -rv, --ros_visualization
                        Publish ros visualization markers.
  -si, --show_images    shows images for each camera
  -oi, --optimize_intrinsics
                        Adds camera instrinsics and distortion to the optimization
  -sr SAMPLE_RESIDUALS, --sample_residuals SAMPLE_RESIDUALS
                        Samples residuals
  -ss SAMPLE_SEED, --sample_seed SAMPLE_SEED
                        Sampling seed
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection
                        for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial. Used or
                        the Charuco.
  -ssf SENSOR_SELECTION_FUNCTION, --sensor_selection_function SENSOR_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that
                        receives a sensor name as input and returns True or
                        False to indicate if the sensor should be loaded (and
                        used in the optimization). The Syntax is lambda name:
                        f(x), where f(x) is the function in python language.
                        Example: lambda name: name in ["left_laser",
                        "frontal_camera"] , to load only sensors left_laser
                        and frontal_camera
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that
                        receives a collection name as input and returns True
                        or False to indicate if the collection should be
                        loaded (and used in the optimization). The Syntax is
                        lambda name: f(x), where f(x) is the function in
                        python language. Example: lambda name: int(name) > 5 ,
                        to load only collections 6, 7, and onward.
```

It is also possible to call some of these through the launch file. Check the launch file to see how.

##### Advanced usage - two stage calibration for robotic systems with an anchored sensor

When one sensor is set to be acnhored in the calibration/config.yml file, i.e. this [file](https://github.com/lardemua/atlascar2/blob/6850dfe2209e3f5e9c7a3ca66a2b98054ebed256/atlascar2_calibration/calibration/config.yml#L99) for the AtlaCar2, we recommend a two stage procedure to achieve a more accurate calibration:

First, run a calibration using parameter **--only_anchored_sensor** (**-oas**) which will exclude from the optimization all sensors which are not the anchored one. This optimization will position the patterns correctly w.r.t. the anchored sensor. For example:

    rosrun atom_calibration calibrate -json $ATOM_DATASETS/larcc_real/ dataset_train/dataset_corrected.json -uic -nig 0.0 0.0 -ipg -si -rv -v -oas

The output is stored in the **atom_calibration.json**, which is used and the input for the second stage, where all sensors are used. In this second stage the poses of the patterns are frozen using the parameter **--anchor_patterns** (**-ap**). To avoid overwritting atom_calibration.json, you should also define the output json file (**-oj**). For example:

    rosrun atom_calibration calibrate -json $ATOM_DATASETS/larcc_real/ dataset_train/atom_calibration.json -uic -nig 0.0 0.0 -ipg -si -rv -v -ap -oj atom_anchored_calibration.json

