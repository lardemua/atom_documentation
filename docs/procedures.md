## System calibration - Detailed Description

To calibrate your robot you must define your robotic system, (e.g. <your_robot\>). You should also have a **system
description** in the form of an [urdf](http://wiki.ros.org/urdf) or a [xacro](http://wiki.ros.org/xacro) file(s). This is normally stored in a ros package named **<your_robot\>_description**.


!!! Note
    We recommend using xacro files instead of urdfs. 

Finally, **ATOM** requires a bagfile with a recording of the data from the sensors you wish to calibrate.

Transformations in the bagfile (i.e. topics /tf and /tf_static) will be ignored, so that they do not collide with the
ones being published by the [robot_state_publisher](http://wiki.ros.org/robot_state_publisher). Thus, if your robotic system contains moving parts, the bagfile should also record the [sensor_msgs/JointState](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/JointState.html) message.

!!! Note
    It is also possible to use the transformations in the bagfile instead of using the xacro description and the robot state publisher to produce them. 
    See section on [configuring a calibration package](#configuring-a-calibration-package).


To reduce the bag size, it may contain compressed images instead of raw images, since **ATOM** can decompress them while playing back the bagfile. 
Here is an example of a [launch file](https://github.com/lardemua/atlascar2/blob/master/atlascar2_bringup/launch/record_sensor_data.launch)
which records compressed images.

We consider this to be part of the normal configuration of your robotic system in ROS, so ATOM assumes this is already done. 
In any case if you need inspiration you can take a look at the [calibration examples](examples.md) and how we configured our systems.


### Creating a calibration package

Assuming you have your robotic system setup, you can start creating the calibration package.
You should create a calibration ros package specific for your robotic system. **ATOM** provides a script for this:

```bash
rosrun atom_calibration create_calibration_pkg --name <your_robot_calibration>
```

This will create the ros package <your_robot_calibration> in the current folder, but you can also specify another folder,
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
a [simulated hand eye system](https://github.com/miguelriemoliveira/mmtbot/blob/main/mmtbot_calibration/calibration/config.yml).
Also, the file contains several comments to provide clues on how to configure it.

After filling the config.yml file, you should run the package configuration:

```bash
rosrun <your_robot_calibration> configure 
```

This will go through a series of varifications, and create a set of files for launching the system, configuring rviz,
etc.

It is also possible to configure your calibration package with a different configuration file, in the case you have
multiple configurations with multiple config.yml files. There are also other options. 
To run a custom configuration you should use:

```bash
rosrun atom_calibration configure_calibration_package  --name <your_robot_calibration> <other options>
```

```bash
usage: rosrun atom_calibration configure_calibration_pkg [-h] -n NAME [-utf] [-cfg CONFIG_FILE]

-h, --help            show this help message and exit
-n NAME, --name NAME  package name
-utf, --use_tfs       Use transformations in the bag file instead of generating new tfs from the xacro,
                      joint_state_msgs and robot state publisher.
-cfg CONFIG_FILE, --config_file CONFIG_FILE
                      Specify if you want to configure the calibration package with a specific configutation file.
                      If this flag is not given, the standard config.yml ill be used.
```

### Set initial estimate

Iterative optimization methods are often sensitive to the initial parameter configuration. There are several optimization parameters. 
However, the ones we refer to in this case are those that represent the poses of each sensor. **ATOM** provides an interactive framework based on rviz which allows the
user to set the pose of the sensors while having immediate visual feedback.

To set an initial estimate run:

```bash
roslaunch <your_robot_calibration> set_initial_estimate.launch 
```

Here are a couple of examples of setting the initial estimate:

<p align="center">
  <img width="100%" src="/img/agrob_initial_estimate.gif">
</p>
<p align = "center">
Setting initial estimate of sensor poses in the AgrobV2.
</p>

<p align="center">
  <img width="100%" src="/img/set_initial_estimate_atlascar2.gif">
</p>
<p align = "center">
Setting initial estimate of sensor poses in the AtlasCar2.
</p>

<p align="center">
  <img width="100%" src="/img/ur10e_eye_in_hand_set_initial_estimate.gif">
</p>
<p align = "center">
Setting initial estimate of sensor poses in the IRIS UR10e.
</p>


### Collect data 

To run a system calibration, one requires data from the sensors collected at different time instants. We refer to these snapshots of data as [collections](concepts.md#collections), and a set of collections as an [ATOM dataset](concepts.md#atom-datasets). 


To collect data, use:

```bash
roslaunch <your_robot_calibration> collect_data.launch output_folder:=$ATOM_DATASETS/<your_dataset_folder>
```

It is also possible to add additional parameters to configure several aspects of the script. See below all the options.

!!! Tip "Additional parameters for collect_data.launch"

    | Argument  | Function | 
    |:---:|:---:|
    |  overwrite | overwrites previous dataset, if existent   |
    | bag_rate | Defines the playback rate of the bagfile | 
    | bag_start | Start time for playback | 
    | bag_file | Name of bagfile to playback | 
    | ssl | A string to be evaluated that indicates if a sensor should be labelled. |

    One example using all the parameters above:

        roslaunch <your_robot_calibration> collect_data.launch output_folder:=$ATOM_DATASETS/<your_dataset_folder> overwrite:=true bag_rate:=0.5 bag_start:=10 ssl:='lambda name: name in ["s1", "s2"]' 

The data collection script starts data labeling processes adequate for each sensor in your robotic system.


Next, some examples of collecting data in several robotic systems.

<p align="center">
  <img width="100%" src="/img/collect_data_atlascar2.gif">
</p>
<p align = "center">
Automatic labelling and creating an ATOM dataset of the AtlasCar2.
</p>

<p align="center">
  <img width="100%" src="/img/agrob_data_collection.gif">
</p>
<p align = "center">
Automatic labelling and creating an ATOM dataset of the AgrobV2.
</p>



[Atlascar2](https://github.com/lardemua/atlascar2)  | [AgrobV2](https://github.com/aaguiar96/agrob) | [UR10e eye to_base](https://github.com/iris-ua/iris_ur10e_calibration)
------------- | ------------- | -------------
<img align="center" src="docs/collect_data_atlascar2.gif" width="450"/>  | <img align="center" src="docs/agrob_data_collection.gif" width="450"/> | <img align="center" src="docs/ur10e_eye_to_base_collect_data.gif" width="450"/>


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

