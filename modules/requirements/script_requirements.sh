IN_USERNAME=$1
IN_SSI_PATH=$2
IN_ROS_VERSION=$3
IN_CUDA_VERSION_NUMBER=$4
IN_USERNAME="${IN_USERNAME:=pha}"
IN_SSI_PATH="${IN_SSI_PATH:=/home/${IN_USERNAME}/docker_share}"
IN_ROS_VERSION="${IN_ROS_VERSION:=humble}"
IN_CUDA_VERSION_NUMBER="${IN_CUDA_VERSION_NUMBER:=11.7}"

IN_CUDA_VERSION=cuda-${IN_CUDA_VERSION_NUMBER}

# Setup Display
echo -e "\n# Setup" >> /home/${IN_USERNAME}/.bashrc
echo "export USER=${IN_USERNAME}" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/scripts/setup/term_disp.sh" >> /home/${IN_USERNAME}/.bashrc

# Setup ROS Workspace
cd /home/${IN_USERNAME}/autoware/src/universe/external
git clone https://github.com/ZATiTech/open_planner -b humble
git clone https://github.com/autowarefoundation/awf_velodyne
cd /home/${IN_USERNAME}/autoware/src
mkdir -p param/autoware_individual_params/individual_params/config/default/carla_sensor_kit
cp universe/external/open_planner/op_carla_bridge/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml param/autoware_individual_params/individual_params/config/default/carla_sensor_kit/sensor_kit_calibration.yaml
cp universe/external/open_planner/op_carla_bridge/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensors_calibration.yaml param/autoware_individual_params/individual_params/config/default/carla_sensor_kit/sensors_calibration.yaml

# Setup Scenario Runner
cd /home/${IN_USERNAME}
source /home/${IN_USERNAME}/.bashrc
curl -O https://www.antlr.org/download/antlr-4.10.1-complete.jar
sudo cp antlr-4.10.1-complete.jar /usr/local/lib/
rm antlr-4.10.1-complete.jar

# Setup Source Paths
echo "source ${IN_SSI_PATH}/scripts/setup/export_docker_paths.sh" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/git_pkgs/pha_addons/pha_carlaware/modules/scripts/export_docker_carlaware.sh" >> /home/${IN_USERNAME}/.bashrc
echo "source ${IN_SSI_PATH}/git_pkgs/pha_addons/pha_carlaware/modules/scripts/export_op.sh" >> /home/${IN_USERNAME}/.bashrc

echo -e "\n# Scenario Runner" >> /home/${IN_USERNAME}/.bashrc
echo 'export CLASSPATH=".:/usr/local/lib/antlr-4.10.1-complete.jar:$CLASSPATH"' >> /home/${IN_USERNAME}/.bashrc
echo "alias antlr4='java -jar /usr/local/lib/antlr-4.10.1-complete.jar'" >> /home/${IN_USERNAME}/.bashrc
echo "alias grun='java org.antlr.v4.gui.TestRig'" >> /home/${IN_USERNAME}/.bashrc

# Nebula Update
cd /home/${IN_USERNAME}/autoware/src/sensor_component/external/nebula/nebula_decoders/include/nebula_decoders/nebula_decoders_velodyne/decoders/
sed -i -e 's+<angles/angles/angles.h>+<angles/angles.h>+g' velodyne_scan_decoder.hpp

# Install Autoware
source /home/${IN_USERNAME}/.bashrc
cd /home/${IN_USERNAME}/autoware
rm -rf /install /log /build

# sudo rosdep init
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro ${IN_ROS_VERSION}
source /opt/ros/${IN_ROS_VERSION}/setup.bash
MAKEFLAGS=-j4 colcon build --parallel-workers=4 --symlink-install --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-${IN_CUDA_VERSION_NUMBER} -DCMAKE_CUDA_COMPILER=/usr/local/cuda-${IN_CUDA_VERSION_NUMBER}/bin/nvcc -DCUDA_ARCHITECTURES=all

# Setup ROS
echo -e "\n# ROS" >> /home/${IN_USERNAME}/.bashrc
echo "source /opt/ros/${IN_ROS_VERSION}/setup.bash" >> /home/${IN_USERNAME}/.bashrc
echo "source /home/${IN_USERNAME}/autoware/install/setup.bash" >> /home/${IN_USERNAME}/.bashrc

# Colcon
echo -e "\n# Colon" >> /home/${IN_USERNAME}/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/${IN_USERNAME}/.bashrc
echo "export _colcon_cd_root=/opt/ros/${IN_ROS_VERSION}/" >> /home/${IN_USERNAME}/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${IN_USERNAME}/.bashrc

# Set tmux
cd /home/${IN_USERNAME}
touch .tmux.conf
echo 'set-option -g default-shell "/bin/bash"' >> .tmux.conf