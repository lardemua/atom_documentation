# ATOM in ROS2

- [ATOM in ROS2](#atom-in-ros2)
  * [Prepare shared directory](#prepare-shared-directory)
  * [Clone required packages](#clone-required-packages)
  * [Prepare Docker container](#prepare-docker-container)
    + [Build Docker Image](#build-docker-image)
    + [Start docker container](#start-docker-container)
  * [Build Atom and dependencies](#build-atom-and-dependencies)
    + [Credits](#credits)

Atom was originally written for ROS1 and doesn't support ROS2 out-of-the-box. But you can use it within a container. You can easily pass your ROS2 bag to Atom, it’ll be seamlessly [converted](https://github.com/lardemua/atom/pull/718) to ROS1 bag. (It can process both ROS1 and ROS2 bags [seamlessly](https://github.com/lardemua/atom/pull/718))

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
cd ~/noetic_shared/calib_ws/src && git clone git@github.com:cmrobotics/atom.git && git clone https://github.com/cmrobotics/rosbags.git
```


## Prepare Docker container

### Build Docker Image

```bash
docker build --tag=ros:noetic <path_to_atom_repo>/dockerfiles/noetic
```

### Start docker container

```jsx
shared_dir="$1"

xhost +
docker container run --rm --privileged -it --network=host --ipc=host \
	--volume=/dev:/dev --volume=${shared_dir}:/home/ros1/ \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix ${shared_credentials} \
	--env="QT_X11_NO_MITSHM=1" --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -u 1000 \
	--name noetic -w /home/ros1 ros:noetic
```

## Build Atom and dependencies

After you have container running, it is time to build the calibration software

For that go to the `calib_ws` directory and run:

```bash
catkin_make
```

That will build every package in the workspace

Additionally you need to build `rosbags` package as it is not a `catkin` package. Go to `rosbag` folder and run:

```bash
python3 -m pip install .
```

Do not forget to source your calibration workspace at the end:

```bash
source devel/setup.bash
```

From that moment you are good to start with [calibration procedure](https://lardemua.github.io/atom_documentation/procedures/).

### Credits

Thanks to [Coalescent Mobile Roboticsdimaxano](https://cm-robotics.com/) for the contribution.