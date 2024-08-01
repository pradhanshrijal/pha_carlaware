# pha_carlaware

This package is dedicated to make the installation of [Carla] and [Autoware] easy. It uses the [PHA Docker Files] to make installations easy with [Docker] Containerization. The installations instructions are provided in three levels according to knowledge requirements.  

**Test System**

- Computation: i9-12900K, GPU 4080
- System Setup: 
    - Ubuntu 22.04, ROS 2 Humble
    - CUDA 11.6, CuDNN 8.4.1.50,
    - TensorRT 8.4.2
    - Python 3.10
- Carla Version: `0.9.15`
- Autoware Version: `humble-2023.10-cuda-amd64`
- PHA Version: `0.2.0`

# Content

- [Requirements](#requirements)
- [Installation L1](#installation-l1)
- [Installation L2](#installation-l2)
- [Installation L3](#installation-l3)
- [Building and Running](#building-and-running)
- [Launching Carlaware](#launching-carlaware)
- [License](#license)
- [Bibliography](#bibliography)
- [Citation](#citation)

# Requirements

This project requires `docker`, `docker-compose`, `nvidia-docker`, `wget` to be installed. `sudo` access in necessary for the automated installation of `gdown`.

# Installation L1

L1 installation only requires the user to run a single script for setup. This setups the [PHA Docker Files] to `/home/${USER}/schreibtisch/pha_docker_files` and `pha_carlaware` as an addon to `/home/${USER}/schreibtisch/pha_docker_files/docker_share/pha_addons/pha_carlaware`.

It takes a while, but it will setup all the required files and weights.

```
wget https://raw.githubusercontent.com/pradhanshrijal/pha_carlaware/master/scripts/setup_cwr.sh
source setup_cwr.sh
```

That's it, all the base requirements for the project are setup.

See [Building and Running](#building-and-running) for further instructions.

# Installation L2

L2 installations builds upon the previous set of instructions to help the user modify some simple variables to navigate the flexibility of the project. [PHA Project] is just bunch of scripts coupled together to create containerized systems. This simple idea can be made very powerful if the installations are handled by multiple scripts that can be modularly plugged in and plugged out.

See [Surfing Carlaware] for understanding the basic idea.

# Installation L3

L3 instructions is very vast. This level expands the users knowledge from introduction to [Docker] and [ROS 2 - Humble] to the usage of [Autoware]. The simple variables tweaking from [Installation L2](#installation-l2) are built upon with an indepth information of the full functionality of PHA and it's components. Read through the [PHA Project] and the works of previous contributors in [Mentions](#mentions).

# Building and Running

This section will help the user start Carlaware. It is divided into three sections: `Start Carla`, `Setup Carlaware` and `Launch ROS`.

> [!TIP]
> Run Carla and Carlaware on separate computers for the best performance.

## Start Carla

A simple run script is provided to run `Carla 0.9.15`.

```
cd $CARLAWARE_PHA/scripts
./carla_run.sh
```

## Setup Carlaware

The whole system can be setup with a single script. For the first run it take a while to complete the setup process.

```
cd $CARLAWARE_PHA/scripts
source run_cwr.sh
```

For systems with multiple GPUs like the [NVIDIA DGX Platform] there is a sample script that run only the first two GPUs `0,1`.

```
cd $CARLAWARE_PHA/scripts
source run_cwr_gpus.sh
```

> [!NOTE]
> Once the setup is complete, running the same command will reinitialize the setup.
> To restart the previous setup (i.e. when the pc is started again), run this command from the second time:
> `docker start pha-carlaware-${USER}`

Now you can enter the setup container:

```
docker exec -it pha-carlaware-${USER}
```

This is a space where all the setup in ready for the user.

## Launch ROS

To Launch the script, you have to go to the respective folder and run the launch script.

```
cd $OP_BRIDGE_ROOT/op_scripts
```

> [!NOTE]
> If you are running the system of Two PCs, change IP of `SIMULATOR_LOCAL_HOST` to that of the Carla PC. 
> Make sure you can ping the other PC.

Launch Carlaware.

```
./cwr_exploration.sh
```

# Mentions

The main works were contributed by [Hatem Darweesh]. See the instructions for `humble`:
- [YT Autoware T1] -> Basic Instruction Video
- [YT Autoware T2] -> Main Instructions for Setup
- [Humble OP Instructions] -> Main Instructions for Autoware and Carla setup
- [Initial OP Bridge] -> Initial Setup of Autoware and Carla

# Related Works

Similar work is done by [TUMFTM/Carla-Autoware-Bridge]. Multiple vehicle launching has been made easy by [Zenoh with Carla and Autoware] but as of the time of writing the system only works with Ubuntu 20.04. 

# Bibliography
- [Autoware]
- [Carla]
- [Docker]
- [Hatem Darweesh]
- [Humble OP Instructions]
- [NVIDIA DGX Platform]
- [PHA Docker Files]
- [PHA Project]
- [ROS 2 - Humble]
- [Surfing Carlaware]
- [TUMFTM/Carla-Autoware-Bridge]
- [YT Autoware T1]
- [YT Autoware T2]
- [Zenoh with Carla and Autoware]

[Autoware]: https://github.com/autowarefoundation/autoware
[Carla]: https://github.com/carla-simulator/carla
[Docker]: https://www.docker.com/
[Hatem Darweesh]: https://github.com/hatem-darweesh
[Humble OP Instructions]: https://github.com/ZATiTech/open_planner/tree/humble/op_carla_bridge
[Initial OP Bridge]: https://github.com/orgs/autowarefoundation/discussions/2828
[NVIDIA DGX Platform]: https://www.nvidia.com/en-us/data-center/dgx-platform/
[PHA Docker Files]: https://github.com/pradhanshrijal/pha_docker_files
[PHA Project]: https://pradhanshrijal.github.io/pha-project/
[ROS 2 - Humble]: https://docs.ros.org/en/humble/index.html
[Surfing Carlaware]: https://pradhanshrijal.github.io/blog/surfing-carlaware/
[TUMFTM/Carla-Autoware-Bridge]: https://github.com/TUMFTM/Carla-Autoware-Bridge
[YT Autoware T1]: https://www.youtube.com/watch?v=EFH-vVxn180&ab_channel=HatemDarweesh
[YT Autoware T2]: https://www.youtube.com/watch?v=dxwwNacez7o&ab_channel=HatemDarweesh
[Zenoh with Carla and Autoware]: https://autoware.org/running-multiple-autoware-powered-vehicles-in-carla-using-zenoh/

# License

`pha_carlaware` is released under the [MIT](LICENSE) license.

# Citation

If you find this project useful in your research, please consider citing the original work:

_CARLA: An Open Urban Driving Simulator_<br>Alexey Dosovitskiy, German Ros,
Felipe Codevilla, Antonio Lopez, Vladlen Koltun; PMLR 78:1-16
[[PDF](http://proceedings.mlr.press/v78/dosovitskiy17a/dosovitskiy17a.pdf)]
[[talk](https://www.youtube.com/watch?v=xfyK03MEZ9Q&feature=youtu.be&t=2h44m30s)]


```
@inproceedings{Dosovitskiy17,
  title = {{CARLA}: {An} Open Urban Driving Simulator},
  author = {Alexey Dosovitskiy and German Ros and Felipe Codevilla and Antonio Lopez and Vladlen Koltun},
  booktitle = {Proceedings of the 1st Annual Conference on Robot Learning},
  pages = {1--16},
  year = {2017}
}
```

Darweesh, Hatem, Eijiro Takeuchi, and Kazuya Takeda."OpenPlanner 2.0: The Portable Open Source Planner for Autonomous Driving Applications." [2021 IEEE Intelligent Vehicles Symposium Workshops (IV Workshop)](https://ieeexplore.ieee.org/document/9669253). July 11-17, 2021. Nagoya, Japan.

```
@INPROCEEDINGS{9669253,
  author={Darweesh, Hatem and Takeuchi, Eijiro and Takeda, Kazuya},
  booktitle={2021 IEEE Intelligent Vehicles Symposium Workshops (IV Workshops)}, 
  title={OpenPlanner 2.0: The Portable Open Source Planner for Autonomous Driving Applications}, 
  year={2021},
  volume={},
  number={},
  pages={313-318},
  keywords={Conferences;Roads;Tutorials;Markov processes;Planning;Trajectory;Autonomous vehicles},
  doi={10.1109/IVWorkshops54471.2021.9669253}
}
```

Darweesh, Hatem, Eijiro Takeuchi, Kazuya Takeda, Yoshiki Ninomiya, Adi Sujiwo, Luis Yoichi Morales, Naoki Akai, Tetsuo Tomizawa, and Shinpei Kato. "Open source integrated planner for autonomous navigation in highly dynamic environments." [Journal of Robotics and Mechatronics 29, no. 4](https://www.fujipress.jp/jrm/rb/robot002900040668/) (2017): 668-684.

```
@article{ Darweesh_2017jrm,
	title={Open Source Integrated Planner for Autonomous Navigation in Highly Dynamic Environments},
	author={Hatem Darweesh and Eijiro Takeuchi and Kazuya Takeda and Yoshiki Ninomiya and Adi Sujiwo and Luis Yoichi Morales and Naoki Akai and Tetsuo Tomizawa and Shinpei Kato},
	journal={Journal of Robotics and Mechatronics},
	volume={29},
	number={4},
	pages={668-684},
	year={2017},
	doi={10.20965/jrm.2017.p0668}
}
```

Darweesh, Hatem, Eijiro Takeuchi, and Kazuya Takeda. "Estimating the Probabilities of Surrounding Vehicles’ Intentions and Trajectories using a Behavior Planner." [International journal of automotive engineering 10.4](https://www.jstage.jst.go.jp/article/jsaeijae/10/4/10_20194117/_article/-char/ja/) (2019): 299-308.

```
@article{Hatem Darweesh201920194117,
  title={Estimating the Probabilities of Surrounding Vehicles’ Intentions and Trajectories using a Behavior Planner},
  author={Hatem Darweesh and Eijiro Takeuchi and Kazuya Takeda},
  journal={International Journal of Automotive Engineering},
  volume={10},
  number={4},
  pages={299-308},
  year={2019},
  doi={10.20485/jsaeijae.10.4_299}
}
```
