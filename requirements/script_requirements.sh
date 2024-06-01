IN_USERNAME=$1
IN_SSI_PATH=$2
IN_ROS_VERSION=$3
IN_CUDA_VERSION_NUMBER=$4
IN_USERNAME="${IN_USERNAME:=pha}"
IN_SSI_PATH="${IN_SCRIPTS_PATH:=/home/${IN_USERNAME}/docker_share}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CUDA_VERSION_NUMBER="${IN_CUDA_VERSION_NUMBER:=11.7}"

IN_CUDA_VERSION=cuda-${IN_CUDA_VERSION_NUMBER}

# Setup Display
echo -e "\n# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/scripts/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup ROS
cd /home/${IN_USERNAME}/autoware/src/universe/external
git clone https://github.com/ZATiTech/open_planner -b humble
git clone https://github.com/autowarefoundation/awf_velodyne
cd cd /home/${IN_USERNAME}/autoware/src
mkdir -p param/autoware_individual_params/individual_params/config/default/carla_sensor_kit
cp universe/external/open_planner/op_carla_bridge/carla_sensor_kit_launch/carla_sensor_kit_description/config/* param/autoware_individual_params/individual_params/config/default/carla_sensor_kit/.

# Setup Source Paths
echo "source ${IN_SSI_PATH}/scripts/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/git_pkgs/pha_addons/pha_carlaware/scripts/export_docker_carlaware.sh" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/git_pkgs/pha_addons/pha_carlaware/scripts/export_op.sh" >> /home/${IN_USERNAME}/.bashrc
echo "source /opt/ros/${IN_ROS_VERSION}/setup.bash" >> /home/${IN_USERNAME}/.bashrc