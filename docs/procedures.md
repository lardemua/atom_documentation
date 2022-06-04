## Calibration procedures

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


### Create a calibration package

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

### Configure a calibration package

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
multiple configurations with multiple config.yml files. There are also other options to run a custom configuration, i.e.:

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

### Set an initial estimate

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

The script launches an rviz window already configured. The user observes the data playback and **decides when a collection should be saved** by clicking a green sphere in that appears in the scene.

It is also possible to add additional parameters to configure several aspects of the script. See below all the options.

!!! Tip "Additional parameters for collect_data.launch"

    | Argument  | Function | 
    |:---:|:---:|
    |  overwrite | overwrites previous dataset without asking for confirmation   |
    | bag_rate | Defines the playback rate of the bagfile | 
    | bag_start | Start time for playback | 
    | bag_file | Name of bagfile to playback | 
    | ssl | A string to be evaluated that indicates if a sensor should be labelled. |

    One example using all the parameters above:

        roslaunch <your_robot_calibration> collect_data.launch output_folder:=$ATOM_DATASETS/<your_dataset_folder> overwrite:=true bag_rate:=0.5 bag_start:=10 ssl:='lambda name: name in ["s1", "s2"]' 





When you launch the data collection script, it automatically starts data labeling processes adequate for each sensor in your robotic system.
As such, the data is being continuously labeled as the bagfile is played.

Depending on the modalidity of the sensors in the system the labelling may be automatic or fully automatic.
Below we detail how each of the labelers operate.

#### RGB camera labeling

RGB cameras have a fully automatic pattern detection. It uses off the shelf [chessboard](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga93efa9b0aa890de240ca32b11253dd4a) or [charuco](https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html) calibration pattern detectors.
ATOM provides an rviz configuration which subscribes annotated images received from the pattern detectors. You can check if the detection is working 
by observing the overlays of top of the images.

<p align="center">
  <img width="100%" src="/img/ur10e_eye_to_base_collect_data.gif">
</p>
<p align = "center">
Setting the seed point in 2D Lidar data for semi-automatic labeling (AtlasCar2).
</p>


!!! Note
    Charuco boards are preferable to chessboard patterns, because of two main reasons: the first is that the charuco detection is more more efficient when compared to the chessboard detection; the second is that the charuco pattern is detected even if it is only partially visible in the image, which is very usefull when the sensors in your system have small overlapping fields of view.

#### 3D Lidar labeling

3D Lidar labelling is a semi-automatic procedure. The idea is that the user moves an rviz marker close to where the pattern is present in the lidar point cloud.  

<p align="center">
  <img width="100%" src="/img/MMTBot3DLidarLabeling.gif">
</p>
<p align = "center">
Setting the seed point in 3D Lidar data for semi-automatic labeling (MMTBot).
</p>

After setting this seed position, the system continues to track the patterns pose over the next frames, even if it moves,
as you can see below:

<p align="center">
  <img width="100%" src="/img/agrob_data_collection.gif">
</p>
<p align = "center">
Automatic 3D Lidar labeling and creating an ATOM dataset (AgrobV2).
</p>

!!! Warning "Tracking limitations"

    The tracking procedure may fail if the pattern is too close to another object, as for example the ground plane. This can be solved by making sure the pattern is sufficiently far from all other objects, or during the dataset playback stage. 

#### Visualizing sensor fustrums

<p align="center">
  <img width="100%" src="/img/MMTBot_fustrum.gif">
</p>
<p align = "center">
Visualizing fustrum of RGB sensors in the MMTBot.
</p>


#### Depth camera labeling

<to be written\>

#### 2D Lidar labeling

The labeling of the 2D Lidars is very similar to the labeling of 3D Lidars. The user sets the seed point where the lidar points are observing the pattern, and then the pattern is tracked.

<p align="center">
  <img width="100%" src="/img/collect_data_atlascar2.gif">
</p>
<p align = "center">
Setting the seed point in 2D Lidar data for semi-automatic labeling (AtlasCar2).
</p>


!!! Warning "May be deprecated"
    The 2D Lidar semi-automatic labeling was last used in 2019, so it may be deprecated. If you are interested on having this functionality create an issue with a request.


### Calibrate 

Finally, a system calibration is called through:

```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=~/datasets/<my_dataset>/dataset.json 
```
You can use a couple of launch file arguments to configure the calibration procedure, as seen below:

!!! Tip "Additional parameters for calibrate.launch"

    | Argument  | Function | 
    |:---:|:---:|
    | use_incomplete_collections | Remove collections which do not have a detection for all sensors | 
    | ssf | A string to be evaluated into a lambda function that receives  a sensor <br> name as input and returns True or False to indicate if the sensor <br>should be used in the optimization |  
    | csf |  A string to be evaluated into a lambda function that receives a <br>collection name as input and returns True or False to indicate <br>if that collection should be used in the optimization.| 

    One example using all the parameters above:

        roslaunch <your_robot_calibration> calibrate.launch dataset_file:=$ATOM_DATASETS/<my_dataset>/dataset.json  use_incomplete_collections:=true ssf:='lambda name: name in ["camera1, "lidar2"]' csf:='lambda name: int(name) < 7'


##### Advanced usage - running calibration script in separate terminal

Alternatively, for debugging the calibrate script it is better not to have it executed with a bunch of other scripts
which is what happens when you call the launch file. You can run everything with the launch excluding without the
calibrate script using the **run_calibration:=false** option, e.g.:

```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=~/datasets/<my_dataset>/dataset.json run_calibration:=false 
```

and then launch the calibrate script in standalone mode:

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

When one sensor is set to be anchored in the calibration/config.yml file, i.e. this [file](https://github.com/lardemua/atlascar2/blob/6850dfe2209e3f5e9c7a3ca66a2b98054ebed256/atlascar2_calibration/calibration/config.yml#L99) for the AtlaCar2, we recommend a two stage procedure to achieve a more accurate calibration:

First, run a calibration using parameter **--only_anchored_sensor** (**-oas**) which will exclude from the optimization all sensors which are not the anchored one. This optimization will position the patterns correctly w.r.t. the anchored sensor. For example:

    rosrun atom_calibration calibrate -json $ATOM_DATASETS/larcc_real/ dataset_train/dataset_corrected.json -uic -nig 0.0 0.0 -ipg -si -rv -v -oas

The output is stored in the **atom_calibration.json**, which is used and the input for the second stage, where all sensors are used. In this second stage the poses of the patterns are frozen using the parameter **--anchor_patterns** (**-ap**). To avoid overwritting atom_calibration.json, you should also define the output json file (**-oj**). For example:

    rosrun atom_calibration calibrate -json $ATOM_DATASETS/larcc_real/ dataset_train/atom_calibration.json -uic -nig 0.0 0.0 -ipg -si -rv -v -ap -oj atom_anchored_calibration.json


