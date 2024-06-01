#!/bin/bash

# Setup Carlaware
# Sample: source setup_cwr.sh /home/pha/schreibtisch

# Variables
PHA_PARENT=$1
PHA_PARENT="${PHA_PARENT:=/home/${USER}/schreibtisch}"
#

# Install Requirements
sudo apt install python3 python3-pip -y
python3 -m pip install gdown
#

# Parent Folder
if [ ! -d "$PHA_PARENT" ]; then
    mkdir -p $PHA_PARENT
fi
#

# PHA Folder
cd $PHA_PARENT

if [ ! -d pha_docker_files ]; then
    git clone https://github.com/pradhanshrijal/pha_docker_files --recursive
fi

cd pha_docker_files

if [[ -z "${PHA_HOME}" ]]; then
    echo "# PHA" >> /home/${USER}/.bashrc
    echo "source ${PHA_PARENT}/pha_docker_files/docker_share/scripts/setup/export_pha.sh" >> /home/${USER}/.bashrc
fi
#

# PHA Carlaware Main Folder
cd ${ADDONS_PHA}

if [ ! -d pha_carlaware ]; then
    git clone https://github.com/pradhanshrijal/pha_carlaware --recursive
fi

if [[ -z "${CARLAWARE_PHA}" ]]: then
    echo "# CARLAWARE" >> /home/${USER}/.bashrc
    echo "source ${ADDONS_PHA}/pha_carlaware/scripts/export_carlaware.sh" >> /home/${USER}/.bashrc
fi
#

# Simulations
cd ${SIMULATORS_PHA}

## Carla
if [ ! -d carla ]; then
    git clone https://github.com/carla-simulator/carla -b 0.9.15.2
fi

cd carla/PythonAPI/carla

if [ ! -d dist ]; then
    mkdir dist
fi

cd dist

if [ ! -f carla-0.9.15-py3.10-linux-x86_64.egg ]; then
    wget https://github.com/gezp/carla_ros/releases/download/carla-0.9.15-ubuntu-22.04/carla-0.9.15-py3.10-linux-x86_64.egg
fi
##

## OpenPlanner Repositories
cd ${SIMULATORS_PHA}

if [ ! -d carla_awr ]; then
    mkdir carla_awr
fi

cd carla_awr

if [ ! -d scenario_runner ]; then
    git clone https://github.com/hatem-darweesh/scenario_runner -b openplanner_carla_bridge
fi

if [ ! -d op_bridge ]; then
    git clone https://github.com/pradhanshrijal/op_bridge -b ros2-humble
fi

if [ ! -d op_agent ]; then
    git clone https://github.com/pradhanshrijal/op_agent -b ros2-humble
fi
##
#

# OpenPlanner Weights

## Darknet Weights
cd ${SIMULATORS_PHA}/carla_awr/op_agent/darknet/tlr

if [ ! -f yolov4-bosch.weights ]; then
    gdown 1HmeHff10EMQf-uvwgBPxIqmSv1rSWhZu
fi
##

## Yolov3 Weights
cd ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents/config/yolov3

if [ ! -f yolov3.weights ]; then
    gdown 1wDOlKaTYvtNxdDf-IvmALx6UZngx19F0
fi
##

## Map Weights
cd ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents/maps

if [ ! -d Town01 ]; then
    cd ${SIMULATORS_PHA}/carla_awr/op_agent/autoware-contents
    rm -rf maps
    gdown 1GBcV9uXAq3QVrLfQdqI0LTIRKms_r8kl
fi
##

# End - OpenPlanner Weights

# Autoware Datasets
## Dataset Path
cd ${SSI_PATH}/files

if [ ! -d datasets ]; then
    mkdir datasets
fi

cd datasets

if [ ! -d autoware_data ]; then
    mkdir autoware_data
fi
##

## yabloc_pose_initializer
cd autoware_data

if [ ! -d yabloc_pose_initializer ]; then
    mkdir yabloc_pose_initializer
fi

cd yabloc_pose_initializer

if [ ! -d saved_model]; then
    wget https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
    tar -xzf resources.tar.gz
    rm -rf resources.tar.gz
fi
##

## image_projection_based_fusion
cd ${SSI_PATH}/files/datasets/autoware_data

if [ ! -d image_projection_based_fusion ]; then
    mkdir image_projection_based_fusion
fi

cd image_projection_based_fusion

if [ ! -f pts_voxel_encoder_pointpainting.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_voxel_encoder_pointpainting.onnx \
        https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_backbone_neck_head_pointpainting.onnx
fi
##

## lidar_apollo_instance_segmentation
cd ${SSI_PATH}/files/datasets/autoware_data

if [ ! -d lidar_apollo_instance_segmentation ]; then
    mkdir lidar_apollo_instance_segmentation
fi

cd lidar_apollo_instance_segmentation

if [ ! -f hdl-64.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vlp-16.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/hdl-64.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vls-128.onnx
fi
##

## lidar_centerpoint
cd ${SSI_PATH}/files/datasets/autoware_data

if [ ! -d lidar_centerpoint ]; then
    mkdir lidar_centerpoint
fi

cd lidar_centerpoint

if [ ! -f pts_voxel_encoder_centerpoint.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx
fi
##

## tensorrt_yolo
cd ${SSI_PATH}/files/datasets/autoware_data

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
fi
##

## tensorrt_yolox
cd ${SSI_PATH}/files/datasets/autoware_data

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
fi
##

## traffic_light_classifier
cd ${SSI_PATH}/files/datasets/autoware_data

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
fi
##

## traffic_light_fine_detector
cd ${SSI_PATH}/files/datasets/autoware_data

if [ ! -d traffic_light_fine_detector ]; then
    mkdir traffic_light_fine_detector
fi

cd traffic_light_fine_detector

if [ ! -f tlr_yolox_s_batch_1.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_labels.txt
fi
##

## traffic_light_ssd_fine_detector
cd ${SSI_PATH}/files/datasets/autoware_data

if [ ! -d traffic_light_ssd_fine_detector ]; then
    mkdir traffic_light_ssd_fine_detector
fi

cd traffic_light_ssd_fine_detector

if [ ! -f mb2-ssd-lite-tlr.onnx ]; then
    wget https://awf.ml.dev.web.auto/perception/models/mb2-ssd-lite-tlr.onnx \
       https://awf.ml.dev.web.auto/perception/models/voc_labels_tl.txt
fi
##

# End - Autoware Datasets

