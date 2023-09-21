## Evaluation Procedures

- [Evaluation Procedures](#evaluation-procedures)
    - [Annotation of rgb images](#annotation-of-rgb-images)
    - [RGB to RGB camera evaluation](#rgb-to-rgb-camera-evaluation)
    - [LiDAR to Depth Camera evaluation](#lidar-to-depth-camera-evaluation)
    - [RGB to Depth camera evaluation](#rgb-to-depth-camera-evaluation)
    - [LiDAR to LiDAR evaluation](#lidar-to-lidar-evaluation)
    - [LiDAR to RGB camera evaluation](#lidar-to-rgb-camera-evaluation)
    - [Point cloud image projection](#point-cloud-image-projection)
    - [Ground truth frame evaluation](#ground-truth-frame-evalution)

After the system is calibrated one common concern is to be able to assess the accuracy of the produced calibration. ATOM provides several evaluation scripts for this purpose.

Unlike ATOM which calibrates all sensors simultaneously, evaluations are performed in pairs of sensors, which facilitates comparisons with other calibration approaches (which are mostly pairwise),
e.g. [opencv's stereo calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html). Thus, there is a different evaluation script for each combination of modalities.

The ATOM evaluation scripts often request for a train dataset and a test dataset. 
The train dataset is the dataset which is used to conduct the calibration, while the test dataset is a dataset which was not used for calibration.
Results are reported for the test dataset using the geometric transformations estimated during the calibration which were copied to the train dataset json.
These are meant to be two separate datasets, but if you can also use a single dataset by indicating it both as the train and the test dataset.
In this case, however, you should be aware that the results are being computed with data that was used for training.

#### Annotation of rgb images

To evaluate calibration between range sensors and cameras, it is necessary to annotate the physical limits on the calibration pattern in the images of the collection, to allow a comparison with physical labelings as measured by range sensors .

``` bash
rosrun atom_evaluation  annotate_pattern_borders_in_rgb.py [-h] -d DATASET_FILE -cs CAMERA_SENSOR [-si] [-ww WINDOW_WIDTH] [-ps POINT_SIZE] [-ppp POINTS_PER_PIXEL]

optional arguments:
  -h, --help            show this help message and exit
  -d DATASET_FILE, --dataset_file DATASET_FILE
                        Json file containing input dataset.
  -cs CAMERA_SENSOR, --camera_sensor CAMERA_SENSOR
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

#### RGB to RGB camera evaluation

Evaluates de camera-to-camera reprojection error with the following metrics:

- X and Y errors
- Root mean squared error
- Translation and rotation errors

```
usage: rgb_to_rgb_evaluation [-h] -train_json TRAIN_JSON_FILE -test_json TEST_JSON_FILE -ss SENSOR_SOURCE -st SENSOR_TARGET [-si] [-sg]

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
  -sg, --save_graphics  Save reprojection error graphics.
```

How to run:

``` bash
rosrun atom_evaluation rgb_to_rgb_evaluation.py -train_json <path_to_train_file> -test_json <path_to_test_file> -ss <source_sensor_name> -ts <target_sensor_name>

```

#### LiDAR to Depth Camera evaluation

How to run:

``` bash
rosrun atom_evaluation lidar_to_depth_evaluation.py -train_json <path_to_train_file> -test_json <path_to_test_file> -cs <camera_sensor_name> -rs <lidar_sensor_name> -si

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing train input dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing test input dataset.
  -ld SOURCE_SENSOR, --lidar_sensor SOURCE_SENSOR
                        Source transformation sensor.
  -ds TARGET_SENSOR, --depth_sensor TARGET_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.

```

#### RGB to Depth camera evaluation

How to run:

``` bash
rosrun atom_evaluation depth_sensor_to_camera_evaluation.py -train_json <path_to_train_file> -test_json <path_to_test_file> -cs <camera_sensor_name> -ds <depth_sensor_name>

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing train input dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing test input dataset.
  -cs SOURCE_SENSOR, --camera_sensor SOURCE_SENSOR
                        Source transformation sensor.
  -ds TARGET_SENSOR, --depth_sensor TARGET_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.

```

#### LiDAR to LiDAR evaluation

How to run:

``` bash
rosrun atom_evaluation lidar_to_lidar.py -train_json <path_to_train_file> -test_json <path_to_test_file> -ld1 <source_lidar_sensor_name> -ld2 <target_lidar_sensor_name>

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing train input dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing test input dataset.
  -ld1 SOURCE_SENSOR, --lidar_sensor_1 SOURCE_SENSOR
                        Source transformation sensor.
  -ld2 TARGET_SENSOR, --lidar_sensor_2 TARGET_SENSOR
                        Target transformation sensor.

```

#### LiDAR to RGB camera evaluation

Evaluates the LiDAR-to-Camera calibration through the reprojection of the pattern limit 3D points into the image using
the following metrics:

- X and Y errors
- Root mean squared error

This process requires the annotation of the pattern limit points in the image.

After annotating once, if the user wish to repeat the process, the saved json file with the annotations can be loaded.
For that the `-ua` flag has to be disabled.

```
usage: range_sensor_to_camera_evaluation.py [-h] -train_json TRAIN_JSON_FILE -test_json TEST_JSON_FILE -ss SOURCE_SENSOR -ts TARGET_SENSOR [-si] -ef EVAL_FILE [-ua]

optional arguments:
  -h, --help            show this help message and exit
  -train_json TRAIN_JSON_FILE, --train_json_file TRAIN_JSON_FILE
                        Json file containing input training dataset.
  -test_json TEST_JSON_FILE, --test_json_file TEST_JSON_FILE
                        Json file containing input testing dataset.
  -ss SOURCE_SENSOR, --source_sensor SOURCE_SENSOR
                        Source transformation sensor.
  -ts TARGET_SENSOR, --target_sensor TARGET_SENSOR
                        Target transformation sensor.
  -si, --show_images    If true the script shows images.
  -ef EVAL_FILE, --eval_file EVAL_FILE
                        Path to file to read and/or write the evaluation data.
  -ua, --use_annotation
                        If true, the limit points will be manually annotated.
```

[//]: # (How to run:)

[//]: # (If the annotation was already once:)

[//]: # ()

[//]: # (``` bash)

[//]: # (rosrun atom_evaluation range_sensor_to_camera_evaluation.py -train_json <path_to_train_json> -test_json <path_to_test_json> -ss <source_sensor_name> -ts <target_sensor_name> -si -ef <path_to_output_annotation_json_file>)

[//]: # (```)

[//]: # ()

[//]: # (If there is no annotation:)

[//]: # ()

[//]: # (``` bash)

[//]: # (rosrun atom_evaluation range_sensor_to_camera_evaluation.py -train_json <path_to_train_json> -test_json <path_to_test_json> -ss <source_sensor_name> -ts <target_sensor_name> -si -ua -ef <path_to_output_annotation_json_file>)

[//]: # (```)

[//]: # ()

[//]: # (For each image in the test dataset the user have to annotate four classes corresponding to each one of the pattern)

[//]: # (sides.)

[//]: # ()

[//]: # (How to annotate:)

[//]: # ()

[//]: # (- **click + s** to add a point)

[//]: # (- add points in one of the four edges)

[//]: # (- **p** to change class &#40;that is, move to the next edge&#41;)

[//]: # (- repeat this for the four classes/edges)

[//]: # (- **space** to go to the next image)

[//]: # ()

[//]: # (The result should be something like this &#40;for each image&#41;:)

[//]: # ()

[//]: # (<img align="center" src="docs/lidar2cam_evaluation.png" width="450"/>)

[//]: # ()

[//]: # ()

[//]: # (<!-- #TODO #422 Daniela, add here instructions for the depht to camera and depth to lidar calibrations!-->)

[//]: # ()

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

#### Ground Truth Frame Evaluation

When using simulation it is possible to know that a given dataset contains "perfect" value for the transformations to be estimated.
Using this, we can run a comparison between the ground_truth dataset and the calibrated dataset, and assess if the estimated transformations are accurate.

To run use:

rosrun atom_evaluation ground_truth_frame_evaluation -train_json <calibrated_dataset> -test_json <ground_truth_dataset>

The script produces a table where you can inspect the errors.
It is also possible to configure which frames are analyzed. Use --help to see the options.
