<p align="center">
  <img width="70%" src="img/atom_logo.png">
</p>

## ATOM 



[ATOM](https://github.com/lardemua/atom) is a calibration framework using **A**tomic **T**ransformations **O**ptimization **M**ethod. 

<p align="center">
<a href="https://github.com/lardemua/atom">https://github.com/lardemua/atom</a>
</p>

It contains a set of calibration tools for multi-sensor, multi-modal, robotic systems, based on the optimization of atomic transformations, as provided by a [ROS](https://www.ros.org/) based robot description. Moreover, it provides several scripts to facilitate all the steps of a calibration procedure.

<!-- ![type:video](https://www.youtube.com/watch?v=4B3X_NsX89M&list=PLQN09mzV5mbI4h5IQt3Eu9kugSk-08mnY&index=8) -->

If this work is helpful for you please cite [these publications](publications.md).


### Multimedia

Take a look at the ATOM [youtube playlist](https://www.youtube.com/watch?v=BYs1-H9vh0s&list=PLQN09mzV5mbI4h5IQt3Eu9kugSk-08mnY).


<p align="center">
<iframe width="672" height="378" src="https://www.youtube.com/embed/4B3X_NsX89M?start=154" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</p>


v<p align="center">
<iframe width="644" height="362" src="https://www.youtube.com/embed/eII_ptyMq5E?start=129" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</p>

### Installation

Clone the atom repository to a directory inside your catkin workspace:

    git clone https://github.com/lardemua/atom

then install requirements.

    sudo pip3 install -r requirements.txt


### Configure environment variables

We often use two enviroment variables to allow for easy cross machine access to bagfiles and datasets. If you want to use these you can also add these lines to your _.bashrc_ or _.zhsrc_, adjusting the paths according to your case:

```bash
export ROS_BAGS="$HOME/<bagfiles"
export ATOM_DATASETS="$HOME/datasets"
```

and then you can refer to these environment variables when providing paths to atom scripts, e.g.:

```bash
roslaunch <your_robot_calibration> calibrate.launch dataset_file:=$ATOM_DATASETS/<my_dataset>/dataset.json
```

and you can also refer to them inside
the [calibration configuration file](https://github.com/lardemua/atlascar2/blob/0c065508f325fb57e0439c1ba2e00f9468cd73e7/atlascar2_calibration/calibration/config.yml#L14)



### Contributors

* Miguel Riem Oliveira - University of Aveiro
* Afonso Castro - University of Aveiro
* Eurico Pedrosa - University of Aveiro
* Tiago Madeira - University of Aveiro
* André Aguiar - INESC TEC
* Daniela Rato - University of Aveiro

### Current Maintainers

* Miguel Riem Oliveira - University of Aveiro
* Daniela Rato - University of Aveiro
* Manuel Gomes - University of Aveiro
