#!/bin/bash
set -e
file_name=$(basename "$0")
script_path=$(dirname "$(readlink -f "$0")")

# environment variables
DOCKER_USER="hkust-swarm"
DOCKER_PASSWORD="HKUSTswarm123"
DOCKER_REGISTRY="192.168.31.14"
daemon_json='''{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "insecure-registries": ["192.168.31.14"]
}'''
OAK_IMG="$DOCKER_REGISTRY/hkustswarm/oak_ffc_img"
OAK_TAG="oak_ffc_img:latest"
D2SLAM_IMG="$DOCKER_REGISTRY/hkustswarm/d2slam:jetson"
D2SLAM_TAG="d2slam:jetson"

workspace=$HOME/workspace

# help function
function usage() {
    echo "Usage: $file_name [options]"
    echo "Options:"
    echo "  -h,     --help                  Show this help message and exit"
    echo "  -s,     --setup                 Setup"
    echo "  -oc,    --overclock_cpu         Overclock CPU"
    echo "  -og,    --overclock_gpu         Overclock GPU"
    echo "  -p,     --pull_images           Pull docker images"
    echo "  -g,     --clone_github          Clone github repositories"
    echo "Example:"
    echo "  $file_name --setup"
}



# setup function
function setup() {
    # source environment
    echo "[$file_name] Sourcing ROS environment"
    source /opt/ros/$ROS_DISTRO/setup.bash

    # install dependencies
    echo "[$file_name] Installing dependencies"
    sudo apt-get update
    sudo apt-get install -y ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras ros-$ROS_DISTRO-mavros-msgs ros-$ROS_DISTRO-mavros-extras ros-$ROS_DISTRO-mavlink
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod +x install_geographiclib_datasets.sh
    sudo bash install_geographiclib_datasets.sh
    rm install_geographiclib_datasets.sh

    # configure docker
    echo "[$file_name] Configuring docker"
    sudo groupadd docker || true
    sudo usermod -aG docker $USER
    sudo bash -c "echo '$daemon_json' > /etc/docker/daemon.json"
    sudo systemctl restart docker
    docker login -u $DOCKER_USER -p $DOCKER_PASSWORD $DOCKER_REGISTRY
}

function pull_images() {
    # pull docker images
    echo "[$file_name] Pulling docker images"
    docker pull $OAK_IMG
    docker tag $OAK_IMG $OAK_TAG
    docker pull $D2SLAM_IMG
    docker tag $D2SLAM_IMG $D2SLAM_TAG
}

function clone_github() {
    # clone github repositories
    echo "[$file_name] Cloning github repositories"
    mkdir -p $workspace
    if [ -d "$workspace/perception-D2SLAM" ]; then
        echo "[$file_name] perception-D2SLAM already exists"
    else
        git clone https://github.com/D2SLAM-Fusion/perception-D2SLAM.git $workspace/perception-D2SLAM
    fi

    if [ -d "$workspace/perception-ONNX-CREStereo-Depth-Estimation" ]; then
        echo "[$file_name] perception-ONNX-CREStereo-Depth-Estimation already exists"
    else
        git clone https://github.com/D2SLAM-Fusion/perception-ONNX-CREStereo-Depth-Estimation.git $workspace/perception-ONNX-CREStereo-Depth-Estimation
    fi

    if [ -d "$workspace/perception-ONNX-HITNET-Stereo-Depth-estimation" ]; then
        echo "[$file_name] perception-ONNX-HITNET-Stereo-Depth-estimation already exists"
    else
        git clone https://github.com/D2SLAM-Fusion/perception-ONNX-HITNET-Stereo-Depth-estimation.git $workspace/perception-ONNX-HITNET-Stereo-Depth-estimation
    fi

    if [ -d "$workspace/configs-drone-swarm" ]; then
        echo "[$file_name] configs-drone-swarm already exists"
    else
        git clone https://github.com/D2SLAM-Fusion/configs-drone-swarm.git $workspace/configs-drone-swarm
    fi
}

# overclock functions
function overclock_cpu() {
    # configure nvpmodel CPUmax
    echo "[$file_name] Configuring nvpmodel CPUmax"
    sudo cp $script_path/overclock/nvpmodel/nvpmodel_CPUmax.conf /etc/nvpmodel.conf
    sudo systemctl restart nvpmodel.service
}

function overclock_gpu() {
    # configure nvpmodel GPUmax
    echo "[$file_name] Configuring nvpmodel GPUmax"
    sudo cp $script_path/overclock/nvpmodel/nvpmodel_GPUmax.conf /etc/nvpmodel.conf
    sudo systemctl restart nvpmodel.service
    sudo cp $script_path/overclock/nvpmodel/overclock.service /etc/systemd/system/overclock.service
    sudo systemctl daemon-reload
    sudo systemctl enable overclock.service
    sudo systemctl start overclock.service
}

# main function
function main() {
    # check if no option is provided
    if [[ $# -eq 0 ]]; then
        echo "[$file_name] No option provided"
        usage
        exit 1
    fi

    # parse multiple options
    while [[ $# -gt 0 ]]; do
        key="$1"
        case $key in
            -h|--help)
                usage
                exit 0
                ;;
            -s|--setup)
                setup
                shift
                ;;
            -oc|--overclock_cpu)
                overclock_cpu
                shift
                ;;
            -og|--overclock_gpu)
                overclock_gpu
                shift
                ;;
            -p|--pull_images)
                pull_images
                shift
                ;;
            -g|--clone_github)
                clone_github
                shift
                ;;
            *)
                echo "[$file_name] Unknown option: $key"
                usage
                exit 1
                ;;
        esac
    done

}

main "$@"