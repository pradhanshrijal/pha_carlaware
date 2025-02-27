#!/bin/bash

# Setup Carlaware
# Sample: source setup_cwr.sh /home/pha/schreibtisch github.com ${SSI_PATH}/files/datasets/autoware_data

# Variables
PHA_PARENT=$1
PHA_PARENT="${PHA_PARENT:=/home/${USER}/schreibtisch}"
PHA_DB=$2
PHA_DB="${PHA_DB:=github.com}"
CWR_DATA=$3
CWR_DATA="${CWR_DATA:=${SSI_PATH}/files/datasets/autoware_data}"
#

# Install Requirements
sudo apt install git git-lfs python3 python3-pip x11-xserver.utils -y
python3 -m pip install gdown==4.6.1
#

# Parent Folder
if [ ! -d "$PHA_PARENT" ]; then
    mkdir -p $PHA_PARENT
    echo "Setup PHA Parent Path: ${PHA_PARENT}"
else
    echo "PHA Parent Path already set: ${PHA_PARENT}"
fi
#

# PHA Folder
cd $PHA_PARENT

if [ ! -d pha_docker_files ]; then
    git clone https://${PHA_DB}/pradhanshrijal/pha_docker_files --recursive
    echo "Setup PHA Docker Files."
else
    echo "PHA Docker Files already set."
fi

cd pha_docker_files

if [[ -z "${PHA_HOME}" ]]; then
    echo -e "\n# PHA" >> /home/${USER}/.bashrc
    echo "source ${PHA_PARENT}/pha_docker_files/docker_share/scripts/setup/export_pha.sh" >> /home/${USER}/.bashrc
    echo 'export PATH="${HOME}/.local/bin:$PATH"' >> /home/${USER}/.bashrc
    source /home/${USER}/.bashrc
    echo "Setup PHA Home Path: ${PHA_HOME}"
else
    echo "PHA Home already set: ${PHA_HOME}"
fi
#

# PHA Carlaware Main Folder
cd ${ADDONS_PHA}

if [ ! -d pha_carlaware ]; then
    git clone https://${PHA_DB}/pradhanshrijal/pha_carlaware --recursive
    echo "Setup PHA CARLAWARE."
else
    echo "PHA CARLAWARE already set."
fi

if [[ -z "${CARLAWARE_PHA}" ]]; then
    echo -e "\n# CARLAWARE" >> /home/${USER}/.bashrc
    echo "source ${ADDONS_PHA}/pha_carlaware/modules/scripts/export_carlaware.sh" >> /home/${USER}/.bashrc
    source /home/${USER}/.bashrc
    echo "Setup CARLAWARE PHA Path: ${CARLAWARE_PHA}"
else
    echo "CARLAWARE PHA Path already set: ${CARLAWARE_PHA}"
fi
#

# Simulations
cd ${SIMULATORS_PHA}

## Carla
if [ ! -d carla ]; then
    git clone https://github.com/carla-simulator/carla -b 0.9.15.2
    echo "Setup Carla Simulator."
else
    echo "Carla Simulator already set."
fi

cd carla/PythonAPI/carla

if [ ! -d dist ]; then
    mkdir dist
    echo "Carla: Create Dist Dir."
else
    echo "Carla: Dist Dir exists."
fi

cd dist

if [ ! -f carla-0.9.15-py3.10-linux-x86_64.egg ]; then
    wget https://github.com/gezp/carla_ros/releases/download/carla-0.9.15-ubuntu-22.04/carla-0.9.15-py3.10-linux-x86_64.egg
    echo "Carla: Download Python Egg."
else
    echo "Carla: Egg exists."
fi
##

## Carla Addons
cd ${SIMULATORS_PHA}

if [ ! -d carla_addons ]; then
    mkdir carla_addons
    echo "Setup Addons for Carla."
else
    echo "Carla Addons folder already set."
fi

cd carla_addons

if [ ! -d scenario_runner ]; then
    git clone https://github.com/carla-simulator/scenario_runner -b v0.9.15
    echo "CWR: Setup Scenario Runner."
else
    echo "CWR: Scenario Runner already set."
fi
##

## OpenPlanner Repositories
cd ${SIMULATORS_PHA}

if [ ! -d carla_awr ]; then
    mkdir carla_awr
    echo "Setup OpenPlanner Simulator."
else
    echo "OpenPlanner Simulator already set."
fi

cd carla_awr

if [ ! -d op_bridge ]; then
    git clone https://${PHA_DB}/pradhanshrijal/op_bridge
    echo "OP: Setup Bridge."
else
    echo "OP: Bridge already set."
fi

if [ ! -d op_agent ]; then
    git clone https://${PHA_DB}/pradhanshrijal/op_agent
    echo "OP: Setup Agent."
else
    echo "OP: Agent already set."
fi
##

## Carla Repositories
cd ${SIMULATORS_PHA}

if [ ! -d carla_addons ]; then
    mkdir carla_addons
    echo "Setup Carla Addons."
else
    echo "Carla Addons already set."
fi

if [ ! -d scenario_runner ]; then
    git clone https://github.com/carla-simulator/scenario_runner -b v0.9.15
    echo "OP: Setup Scenario Runner."
else
    echo "OP: Scenario Runner already set."
fi

##
#

# OpenPlanner Weights

echo "Setup Autoware Weights."

## Darknet Weights
cd ${SIMULATORS_PHA}/carla_awr/op_agent/darknet/tlr

if [ ! -f yolov4-bosch.weights ]; then
    gdown 1HmeHff10EMQf-uvwgBPxIqmSv1rSWhZu
    echo "AW: Download 'yolov4-bosch.weights'."
else
    echo "AW: 'yolov4-bosch.weights' exists."
fi
##

## Yolov3 Weights
cd ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents/config/yolov3

if [ ! -f yolov3.weights ]; then
    gdown 1wDOlKaTYvtNxdDf-IvmALx6UZngx19F0
    echo "AW: Download 'yolov3.weights'."
else
    echo "AW: 'yolov3.weights' exists."
fi
##

## Map Weights
cd ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents/maps/Town01

if [ ! -f pointcloud_map.pcd ]; then
    cd ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents
    rm -rf maps
    gdown --folder https://drive.google.com/drive/folders/1GBcV9uXAq3QVrLfQdqI0LTIRKms_r8kl
    echo "AW: Download Maps: ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents/maps"
else
    echo "AW: Maps exists: ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents/maps"
fi
##

# End - OpenPlanner Weights

# Autoware Datasets
## Dataset Path

if [ ! -d "$CWR_DATA" ]; then
    mkdir -p ${CWR_DATA}
    echo "AW: Setup Dataset Path: ${CWR_DATA}"
else
    echo "AW: Dataset Path already set: ${CWR_DATA}"
fi
##

## yabloc_pose_initializer
cd ${CWR_DATA}

if [ ! -d yabloc_pose_initializer ]; then
    mkdir yabloc_pose_initializer
fi

cd yabloc_pose_initializer

if [ ! -d saved_model ]; then
    wget https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
    tar -xzf resources.tar.gz
    rm -rf resources.tar.gz
    echo "AW: Download 'yabloc_pose_initializer'."
else
    echo "AW: 'yabloc_pose_initializer' exists."
fi
##

## image_projection_based_fusion
cd ${CWR_DATA}

if [ ! -d image_projection_based_fusion ]; then
    mkdir image_projection_based_fusion
fi

cd image_projection_based_fusion

if [ ! -f pts_voxel_encoder_pointpainting.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_voxel_encoder_pointpainting.onnx \
        https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_backbone_neck_head_pointpainting.onnx
    echo "AW: Download 'image_projection_based_fusion'."
else
    echo "AW: 'image_projection_based_fusion' exists."
fi
##

## lidar_apollo_instance_segmentation
cd ${CWR_DATA}

if [ ! -d lidar_apollo_instance_segmentation ]; then
    mkdir lidar_apollo_instance_segmentation
fi

cd lidar_apollo_instance_segmentation

if [ ! -f hdl-64.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vlp-16.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/hdl-64.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vls-128.onnx
    echo "AW: Download 'lidar_apollo_instance_segmentation'."
else
    echo "AW: 'lidar_apollo_instance_segmentation' exists."
fi
##

## lidar_centerpoint
cd ${CWR_DATA}

if [ ! -d lidar_centerpoint ]; then
    mkdir lidar_centerpoint
fi

cd lidar_centerpoint

if [ ! -f pts_voxel_encoder_centerpoint.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx
    echo "AW: Download 'lidar_centerpoint'."
else
    echo "AW: 'lidar_centerpoint' exists."
fi
##

## tensorrt_yolo
cd ${CWR_DATA}

if [ ! -d tensorrt_yolo ]; then
    mkdir tensorrt_yolo
fi

cd tensorrt_yolo

if [ ! -f yolov3.onnx ]; then
    wget  https://awf.ml.dev.web.auto/perception/models/yolov3.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5s.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5m.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5l.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5x.onnx \
       https://awf.ml.dev.web.auto/perception/models/coco.names
    echo "AW: Download 'tensorrt_yolo'."
else
    echo "AW: 'tensorrt_yolo' exists."
fi
##

## tensorrt_yolox
cd ${CWR_DATA}

if [ ! -d tensorrt_yolox ]; then
    mkdir tensorrt_yolox
fi

cd tensorrt_yolox

if [ ! -f yolox-tiny.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/yolox-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.onnx \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/label.txt
    echo "AW: Download 'tensorrt_yolox'."
else
    echo "AW: 'tensorrt_yolox' exists."
fi
##

## traffic_light_classifier
cd ${CWR_DATA}

if [ ! -d traffic_light_classifier ]; then
    mkdir traffic_light_classifier
fi

cd traffic_light_classifier

if [ ! -f traffic_light_classifier_mobilenetv2_batch_1.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/lamp_labels.txt
    echo "AW: Download 'traffic_light_classifier'."
else
    echo "AW: 'traffic_light_classifier' exists."
fi
##

## traffic_light_fine_detector
cd ${CWR_DATA}

if [ ! -d traffic_light_fine_detector ]; then
    mkdir traffic_light_fine_detector
fi

cd traffic_light_fine_detector

if [ ! -f tlr_yolox_s_batch_1.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_labels.txt
    echo "AW: Download 'traffic_light_fine_detector'."
else
    echo "AW: 'traffic_light_fine_detector' exists."
fi
##

## traffic_light_ssd_fine_detector
cd ${CWR_DATA}

if [ ! -d traffic_light_ssd_fine_detector ]; then
    mkdir traffic_light_ssd_fine_detector
fi

cd traffic_light_ssd_fine_detector

if [ ! -f mb2-ssd-lite-tlr.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/mb2-ssd-lite-tlr.onnx \
       https://awf.ml.dev.web.auto/perception/models/voc_labels_tl.txt
    echo "AW: Download 'traffic_light_ssd_fine_detector'."
else
    echo "AW: 'traffic_light_ssd_fine_detector' exists."
fi
##

# End - Autoware Datasets

echo "Fin."

cd ${PHA_HOME}