## Evaluation Procedures

- [Evaluation Procedures](#evaluation-procedures)
    - [Annotation of rgb images](#annotation-of-rgb-images)
    - [Full evaluation](#full-evaluation)
    - [LiDAR to Depth evaluation](#lidar-to-depth-evaluation)
    - [LiDAR to LiDAR evaluation](#lidar-to-lidar-evaluation)
    - [LiDAR to RGB evaluation](#lidar-to-rgb-evaluation)
    - [Depth to Depth evaluation](#depth-to-depth-evaluation)
    - [Depth to RGB evaluation](#depth-to-rgb-evaluation)
    - [RGB to RGB evaluation](#rgb-to-rgb-evaluation)
    - [Point cloud image projection](#point-cloud-image-projection)
    - [Ground truth frame evaluation](#ground-truth-frame-evaluation)

After the system is calibrated one common concern is to be able to assess the accuracy of the produced calibration. ATOM provides several evaluation scripts for this purpose.

Unlike ATOM which calibrates all sensors simultaneously, evaluations are performed in pairs of sensors, which facilitates comparisons with other calibration approaches (which are mostly pairwise),
e.g. [opencv's stereo calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html). Thus, there is a different evaluation script for each combination of modalities.

Every sensor evaluation script has two different variations: 

- Intra-collection evaluation: This type of evaluation involves comparing the detections made by a pair of sensors within the same collection. It is utilized to assess the performance of sensor to sensor calibration.

- Inter-collection evaluation: In this type of evaluation, we compare the detections made by a pair of sensors in different collections. To do this, we project the detections from the source sensor's frame to the world frame of the source collection and then project them to the target sensor's frame in the target collection. It evaluates not only the sensor to sensor calibration but also the sensor to frame calibration.

The ATOM evaluation scripts often request for a train dataset and a test dataset. 
The train dataset is the dataset which is used to conduct the calibration, while the test dataset is a dataset which was not used for calibration.
Results are reported for the test dataset using the geometric transformations estimated during the calibration which were copied to the train dataset json.
These are meant to be two separate datasets, but if you can also use a single dataset by indicating it both as the train and the test dataset.
In this case, however, you should be aware that the results are being computed with data that was used for training.


#### Annotation of rgb images

To evaluate calibration between range sensors and cameras, it is necessary to annotate the physical limits on the calibration pattern in the images of the collection, to allow a comparison with physical labelings as measured by range sensors .

``` bash
rosrun atom_evaluation  annotate_pattern_borders_in_rgb -d <path_to_train_file> -cs <rgb_sensor_name> 

optional arguments:
  -h, --help            show this help message and exit
  -d DATASET_FILE, --dataset_file DATASET_FILE
                        Json file containing input dataset.
  -cs RGB_SENSOR, --rgb_sensor RGB_SENSOR
                        Source transformation sensor.
  -si, --show_images    If true the script shows images.
  -ww WINDOW_WIDTH, --window_width WINDOW_WIDTH
                        Width of the window.
  -ps POINT_SIZE, --point_size POINT_SIZE
                        Size of points to draw on image.
  -ppp POINTS_PER_PIXEL, --points_per_pixel POINTS_PER_PIXEL
                        How many points per pixel to sample between annotated points.

```

Note: you must annotate each camera sensor present in your calibration system. These annotation will be used to evaluate both the lidar-camera pairs and depth-camera.

How to annotate:

- **click** to add points in the currently selected pattern side (text in top left corner)
- if the pattern limit is viewed as a straight line in the image you may click only the corners, if needed you can click more points
- once a side is complete, move on to the **n**ext side by pressing "n"
- when the four sides are complete move to the next collection image by pressing "."

The complete list of keys is printed when "h" is pressed. Be sure to label the corners in both intersected edges, ie, each corner should have two different coloured points.


The result should be something like this (for each image):

<figure markdown align=center>
  ![Image title](img/annotation_rgb_images.png){width="90%" }
  <figcaption align=center>Annotation of the boundaries of the pattern in RGB images.</figcaption>
</figure>

Here is a [video tutorial](https://www.youtube.com/watch?v=DSYyKU-nDcs).

#### Full evaluation

During its configuration, ATOM generates a launch file named `full_evaluation.launch`.
This file configures every possible evaluation the system might have and carries it on. 
To run it:
```bash
roslaunch softbot2_calibration full_evaluation.launch train_json:=<path_to_train_file> test_json:=<path_to_test_file>
```

#### LiDAR to Depth evaluation

How to run:

-  Intra-collection

``` bash
rosrun atom_evaluation lidar_to_depth_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ds <depth_sensor_name> -rs <lidar_sensor_name> -sfr

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -rs RANGE_SENSOR, --range_sensor RANGE_SENSOR
                        Source transformation sensor.
  -ds DEPTH_SENSOR, --depth_sensor DEPTH_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -bt BORDER_TOLERANCE, --border_tolerance BORDER_TOLERANCE
                        Define the percentage of pixels to use to create a
                        border. Lidar points outside that border will not
                        count for the error calculations
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that
                        receives a collection name as input and returns True
                        or False to indicate if the collection should be
                        loaded (and used in the optimization). The Syntax is
                        lambda name: f(x), where f(x) is the function in
                        python language. Example: lambda name: int(name) > 5
                        , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection
                        for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or
                        the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_
                        json/results/{name_of_dataset}_{sensor_source}_to_{se
                        nsor_target}_results.csv
```
-  Inter-collection

``` bash
rosrun atom_evaluation inter_collection_lidar_to_depth_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ds <depth_sensor_name> -rs <lidar_sensor_name> -wf <world_frame> -sfr

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -rs RANGE_SENSOR, --range_sensor RANGE_SENSOR
                        Source transformation sensor.
  -ds DEPTH_SENSOR, --depth_sensor DEPTH_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -wf WORLD_FRAME, --world_frame WORLD_FRAME
                        Fixed frame between collections.
  -bt BORDER_TOLERANCE, --border_tolerance BORDER_TOLERANCE
                        Define the percentage of pixels to use to create a
                        border. Lidar points outside that border will not
                        count for the error calculations
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that
                        receives a collection name as input and returns True
                        or False to indicate if the collection should be
                        loaded (and used in the optimization). The Syntax is
                        lambda name: f(x), where f(x) is the function in
                        python language. Example: lambda name: int(name) > 5
                        , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection
                        for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or
                        the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_
                        json/results/{name_of_dataset}_inter_collection_{sensor_source}_to_{se
                        nsor_target}_results.csv
```

#### LiDAR to LiDAR evaluation

How to run:

- Intra-collection:

``` bash
rosrun atom_evaluation lidar_to_lidar_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ss <source_lidar_sensor_name> -st <target_lidar_sensor_name> -sfr

optional arguments:
 -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -ss SENSOR_SOURCE, --sensor_source SENSOR_SOURCE
                        Source transformation sensor.
  -st SENSOR_TARGET, --sensor_target SENSOR_TARGET
                        Target transformation sensor.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_
                        json/results/{name_of_dataset}_{sensor_source}_to_{se
                        nsor_target}_results.csv

```

- Inter-collection:

``` bash
rosrun atom_evaluation inter_collection_lidar_to_lidar_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ss <source_lidar_sensor_name> -st <target_lidar_sensor_name> -wf <world_frame> -sfr

optional arguments:
 -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -ss SENSOR_SOURCE, --sensor_source SENSOR_SOURCE
                        Source transformation sensor.
  -st SENSOR_TARGET, --sensor_target SENSOR_TARGET
                        Target transformation sensor.
  -wf WORLD_FRAME, --world_frame WORLD_FRAME
                        Fixed frame between collections.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_
                        json/results/{name_of_dataset}_inter_collection{sensor_source}_to_{se
                        nsor_target}_results.csv

```

#### LiDAR to RGB evaluation

Evaluates the LiDAR-to-Camera calibration through the reprojection of the pattern limit 3D points into the image using
the following metrics:

- X and Y errors
- Root mean squared error

This process requires the annotation of the pattern limit points in the image.

How to run:

- Intra-collection:

```bash
rosrun atom_evaluation lidar_to_rgb_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -rs <lidar_sensor_name> -cs <rgb_sensor_name> -sfr

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -rs RANGE_SENSOR, --range_sensor RANGE_SENSOR
                        Source transformation sensor.
  -cs CAMERA_SENSOR, --camera_sensor CAMERA_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_{sensor_source}_to_{sensor_target}_results.csv

```

- Inter-collection:

```bash
rosrun atom_evaluation inter_collection_lidar_to_rgb_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -rs <lidar_sensor_name> -cs <rgb_sensor_name> -wf <world_frame> -sfr

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -rs RANGE_SENSOR, --range_sensor RANGE_SENSOR
                        Source transformation sensor.
  -cs CAMERA_SENSOR, --camera_sensor CAMERA_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -wf WORLD_FRAME, --world_frame WORLD_FRAME
                        Fixed frame between collections.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_inter_collection_{sensor_source}_to_{sensor_target}_results.csv

```

#### Depth to Depth evaluation

How to run:

- Intra-collection:

```bash
rosrun atom_evaluation depth_to_depth_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ss <source_depth_sensor_name> -st <target_depth_sensor_name> -sfr

  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -ss SENSOR_SOURCE, --sensor_source SENSOR_SOURCE
                        Source transformation sensor.
  -st SENSOR_TARGET, --sensor_target SENSOR_TARGET
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -bt BORDER_TOLERANCE, --border_tolerance BORDER_TOLERANCE
                        Define the percentage of pixels to use to create a border. Lidar points outside that border will not count for the error calculations
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_{sensor_source}_to_{sensor_target}_results.csv

```

- Inter-collection:

```bash
rosrun atom_evaluation inter_collection_depth_to_depth_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ss <source_depth_sensor_name> -st <target_depth_sensor_name> -wf <world_frame> -sfr

  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -ss SENSOR_SOURCE, --sensor_source SENSOR_SOURCE
                        Source transformation sensor.
  -st SENSOR_TARGET, --sensor_target SENSOR_TARGET
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -wf WORLD_FRAME, --world_frame WORLD_FRAME
                        Fixed frame between collections.
  -bt BORDER_TOLERANCE, --border_tolerance BORDER_TOLERANCE
                        Define the percentage of pixels to use to create a border. Lidar points outside that border will not count for the error calculations
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_inter_collection_{sensor_source}_to_{sensor_target}_results.csv

```

#### Depth to RGB evaluation

How to run:

- Intra-collection:

``` bash
rosrun atom_evaluation depth_to_rgb_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ds <depth_sensor_name> -cs <rgb_sensor_name> -sfr 

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -ds DEPTH_SENSOR, --depth_sensor DEPTH_SENSOR
                        Source transformation sensor.
  -cs CAMERA_SENSOR, --camera_sensor CAMERA_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_{sensor_source}_to_{sensor_target}_results.csv

```

- Inter-collection:

``` bash
rosrun atom_evaluation inter_collection_depth_to_rgb_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ds <depth_sensor_name> -cs <rgb_sensor_name> -wf <world_frame> -sfr 

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -ds DEPTH_SENSOR, --depth_sensor DEPTH_SENSOR
                        Source transformation sensor.
  -cs CAMERA_SENSOR, --camera_sensor CAMERA_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -wf WORLD_FRAME, --world_frame WORLD_FRAME
                        Fixed frame between collections.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_inter_collection_{sensor_source}_to_{sensor_target}_results.csv

```


#### RGB to RGB evaluation

Evaluates de camera-to-camera reprojection error with the following metrics:

- X and Y errors
- Root mean squared error
- Translation and rotation errors

How to run:

- Intra collection:

```bash
rosrun atom_evaluation rgb_to_rgb_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ss <source_rgb_sensor_name> -st <target_rgb_sensor_name> -sfr

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing train input dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing test input dataset.
  -ss SENSOR_SOURCE, --sensor_source SENSOR_SOURCE
                        Source transformation sensor.
  -st SENSOR_TARGET, --sensor_target SENSOR_TARGET
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_{sensor_source}_to_{sensor_target}_results.csv
```

- Inter collection:

```bash
rosrun atom_evaluation inter_collection_rgb_to_rgb_evaluation -train_json <path_to_train_file> -test_json <path_to_test_file> -ss <source_rgb_sensor_name> -st <target_rgb_sensor_name> -wf <world_frame> -sfr

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing train input dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing test input dataset.
  -ss SENSOR_SOURCE, --sensor_source SENSOR_SOURCE
                        Source transformation sensor.
  -st SENSOR_TARGET, --sensor_target SENSOR_TARGET
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -wf WORLD_FRAME, --world_frame WORLD_FRAME
                        Fixed frame between collections.
  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION
                        A string to be evaluated into a lambda function that receives a collection name as input and returns True or False to indicate if the
                        collection should be loaded (and used in the optimization). The Syntax is lambda name: f(x), where f(x) is the function in python
                        language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.
  -uic, --use_incomplete_collections
                        Remove any collection which does not have a detection for all sensors.
  -rpd, --remove_partial_detections
                        Remove detected labels which are only partial.Used or the Charuco.
  -sfr, --save_file_results
                        Store the results
  -sfrn SAVE_FILE_RESULTS_NAME, --save_file_results_name SAVE_FILE_RESULTS_NAME
                        Name of csv file to save the results. Default: -test_json/results/{name_of_dataset}_inter_collection_{sensor_source}_to_{sensor_target}_results.csv
```

#### Point cloud image projection

`atom_evaluation` also allows the user to visualize the point cloud projected into an image to check the calibration.

``` bash
usage: point_cloud_to_image.py [-h] -json JSON_FILE -ls LIDAR_SENSOR -cs CAMERA_SENSOR
optional arguments:
  -h, --help            show this help message and exit
  -json JSON_FILE, --json_file JSON_FILE
                        Json file containing input dataset.
  -ls LIDAR_SENSOR, --lidar_sensor LIDAR_SENSOR
                        LiDAR sensor name.
  -cs CAMERA_SENSOR, --camera_sensor CAMERA_SENSOR
                        Camera sensor name.
```

How to run:

``` bash
rosrun atom_evaluation point_cloud_to_image.py -json <path_to_test_json> -ls <lidar_sensor_name> -cs <camera_sensor_name>
```

#### Ground truth frame evaluation

When using simulation it is possible to know that a given dataset contains "perfect" value for the transformations to be estimated.
Using this, we can run a comparison between the ground_truth dataset and the calibrated dataset, and assess if the estimated transformations are accurate.

To run use:

    rosrun atom_evaluation ground_truth_frame_evaluation -train_json <calibrated_dataset> -test_json <ground_truth_dataset>

The script produces a table where you can inspect the errors.
It is also possible to configure which frames are analyzed. Use --help to see the options.
