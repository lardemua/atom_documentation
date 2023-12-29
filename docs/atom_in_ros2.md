# ATOM in ROS2

- [ATOM in ROS2](#atom-in-ros2)
  * [Prepare shared directory](#prepare-shared-directory)
  * [Clone required packages](#clone-required-packages)
  * [Prepare Docker container](#prepare-docker-container)
    + [Build Docker Image](#build-docker-image)
    + [Start docker container](#start-docker-container)
  * [Build Atom and dependencies](#build-atom-and-dependencies)
  + [Credits](#credits)

Atom was originally written for ROS1 and doesn't support ROS2 out-of-the-box. But the good news is it can be used within a container. You can easily pass your ROS2 bag to Atom, it’ll be seamlessly [converted](https://github.com/lardemua/atom/pull/718) to ROS1 bag.

## Prepare shared directory

All the code, datasets and rosbags will be stored in the shared directory i.e. the directory you can access from both host (your computer) and the container. In this tutorial, we place a shared directory in your computer's home folder, but you are free to place it anywhere.

```bash
mkdir -p ~/noetic_shared_dir/calib_ws/src && mkdir -p ~/noetic_shared_dir/datasets && mkdir -p ~/noetic_shared_dir/bagfiles
```

## Clone required packages

We need clone several packages into the workspace source folder:

- `Atom` calibration package
- `rosbags` packages. Required to perform a ROS2-to-ROS1 bag conversion

```bash
cd ~/noetic_shared_dir/calib_ws/src && git clone git@github.com:lardemua/atom.git && git clone https://github.com/cmrobotics/rosbags.git
```

## Prepare Docker container

### Build Docker Image

```bash
docker build --tag=ros:noetic ~/noetic_shared_dir/calib_ws/src/atom/dockerfiles/noetic
```

### Start docker container

```bash
xhost + && docker container run --rm --privileged -it --network=host --ipc=host \
--volume=/dev:/dev --volume=$HOME/noetic_shared_dir:/home/ros1/ \
--volume=/tmp/.X11-unix:/tmp/.X11-unix \
--env="QT_X11_NO_MITSHM=1" --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -u 1000 \
--name noetic -w /home/ros1 ros:noetic
```

## Build Atom and dependencies

After you have container running, it is time to build the calibration software.

First, you need to build `rosbags` package as it is not a `catkin` package. Go to `rosbag` folder and run:

```bash
python3 -m pip install .
```

To install the rest of the packages go to the `calib_ws` directory and run:

```bash
catkin_make
```

That will build every package in the workspace.

Do not forget to source your calibration workspace at the end:

```bash
source devel/setup.bash
```

## Calibration

Mostly, the calibration process for ROS2 bag shares steps with ROS1 [calibration](https://lardemua.github.io/atom_documentation/procedures/).

The only difference comes from the fact that we are unable to build ROS2 robot description packages in ROS1 container, so we need to pass robot model file as an absolute path.

1. Keep in mind while preparing your calibration config file that `description_file` should be an absolute path to your robot model, not a relative one.

2. Also pass path to the robot description file  (`description_file:=<path_to_robot_model>`) alognside with other arguments in [Collect Data](https://lardemua.github.io/atom_documentation/procedures/#collect-data) step.

## Credits

Thanks to [Coalescent Mobile Robotics](https://github.com/cmrobotics) for the contribution.