#!/bin/bash
set -e
file_name=$(basename "$0")
script_path=$(dirname "$(readlink -f "$0")")

function usage() {
    echo "Usage: $file_name [options]"
    echo "Options:"
    echo "  -h,     --help                  show this help message and exit"
    echo "  -s,     --setup                 Setup"
    echo "  -oc,    --overclock_cpu         Overclock CPU"
    echo "  -og,    --overclock_gpu         verclock GPU"
    echo "Example:"
    echo "  $file_name --setup"
}

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
}

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
            *)
                echo "[$file_name] Unknown option: $key"
                usage
                exit 1
                ;;
        esac
    done

}

main "$@"