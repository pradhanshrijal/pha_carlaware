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

[PHA Project] is just bunch of scripts coupled together to create containerized systems. This simple idea can be made very powerful if the installations are handled by multiple scripts that can be modularly plugged in and plugged out. L2 installations builds upon the previous set of instructions to help the user modify some simple variables to navigate the flexibility of the project.

See []() for understanding the basic idea with a simple example.

# Installation L3

# Building and Running

This section will help the user start Carlaware. It is divided into three sections: `Start Carla`, `Start Carlaware`, `Launch ROS`.

> [!TIP]
> Run Carla and Carlaware on separate computers for the best performance.

# Launching Carlaware

# Mentions

TODO: Hatem Darwesh

# Related Works

# Bibliography
- [Autoware]
- [Carla]
- [Docker]
- [PHA Docker Files]
- [PHA Project]

[Autoware]: https://github.com/autowarefoundation/autoware
[Carla]: https://github.com/carla-simulator/carla
[Docker]: https://www.docker.com/
[PHA Docker Files]: https://github.com/pradhanshrijal/pha_docker_files
[PHA Project]: https://pradhanshrijal.github.io/pha-project/

# License

`pha_carlaware` is released under the [MIT](LICENSE) license.

# Citation

If you find this project useful in your research, please consider citing the original works: