## Concepts

Here we describe the main concepts in ATOM.

### Collections

A collection is a recording of the data from all the sensors in the system at a particular time instant selected by the user. Collections also contain information about labels for the sensor data, as well as the state of the robotic system at that time, i.e., all the transformations.

### ATOM Datasets

An ATOM dataset is a folder which contains data used for the calibration of a robotic system. Every ATOM dataset contains a **dataset.json** which provides details about the dataset, such as the defined configuration, the number of sensors, etc. 

Several scripts in the calibration pipeline require an ATOM dataset, but is worth mentioning that the files are also human readable. 

Below you can see the structure of an ATOM dataset.


<p align="center">
<img width="50%" src="/img/ATOMDataset_tree.png">
</p>
<p align = "center">
Structure of an ATOM dataset json file.
</p>

A **dataset.json** file contains a **_metadata** field, where details about the date, user and others are stored. It also contains a **calibration_config**, which is a copy of the state of the configuration file at the time of creation of the dataset. The **sensors** field describes each of the sensors in the system, in particular those selected to be calibrated. 

Finally, the **collections** field contains several collections, i.e. snapshots of data. Each collection contains a subfield **data**, that stores a dictionary obtained through the [conversion of the ROS message to a python dictionary](http://wiki.ros.org/rospy_message_converter), the subfield **labels** contains information about annotations (automatic or manual) of each sensor data, and the subfield **transforms** contains all the transformations published at (or near) the time of the collection.

In addition to the **dataset.json** file, ATOM datasets also contain dedicated files for larger data blobs, such as point clouds or images are saved separately in the same folder.

Because the transformations are stored for each collection, it is possible recover the complete state of the robotic system at the time of each collection. Below we view the different poses of the manipulator and the calibration pattern for each collection.

<p align="center">
<img width="90%" src="/img/mmtbot_multiple_collections4.png">
</p>
<p align = "center">
Several collections in an MMTBot dataset.
</p>

