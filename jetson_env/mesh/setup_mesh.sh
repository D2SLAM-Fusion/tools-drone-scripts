#!/bin/bash

set -e

# Enable "error-exit" mode, which exits the script immediately if any command fails.

load_batman_adv() {
    # Clone the batman-adv repository from the specified URL to /tmp/batman-adv directory
    git clone https://git.open-mesh.org/batman-adv.git /tmp/batman-adv

    # Change the working directory to /tmp/batman-adv
    cd /tmp/batman-adv

    # Install batman-adv by running the 'make install' command with multi-threading support
    make -j$(nproc) && sudo make install

    # add batman-adv to /etc/modules
    echo "batman_adv" | sudo tee -a /etc/modules

    # Load the batman-adv kernel module
    sudo modprobe batman_adv

    # Remove the /tmp/batman-adv directory
    rm -rf /tmp/batman-adv

    # Print a message indicating that batman_adv is loaded
    echo "batman_adv is loaded."
}

install_batctl() {
    # Update the package list
    sudo apt-get update

    # Install the batctl package
    sudo apt-get install -y batctl

    # Print a message indicating that batctl is installed
    echo "batctl is installed."
}

config_wlan0() {
    # Stop the network-manager service
    sudo service network-manager stop

    # Block the wifi interface
    sudo rfkill block wifi

    # Unblock the wlan0 interface
    sudo rfkill unblock wifi

    # Set the wlan0 interface down
    sudo ip link set wlan0 down

    # Pause for 5 second
    sleep 5

    # Flush the IP addresses assigned to wlan0
    sudo ip addr flush dev wlan0

    # Set the wlan0 interface mode to ad-hoc
    sudo iwconfig wlan0 mode ad-hoc

    # Set the ESSID of wlan0 to "uav-swarm"
    sudo iwconfig wlan0 essid uav-swarm

    # Set the channel of wlan0 to 2.412GHz
    sudo iwconfig wlan0 freq 2.412G

    # Pause for 1 second
    sleep 1

    # Set the wlan0 interface up
    sudo ip link set wlan0 up

    # Print a message indicating that wlan0 is configured
    echo "wlan0 is configured."
}

config_bat0() {
    # Set the bat0 interface down
    if ifconfig bat0 >/dev/null 2>&1; then
        sudo ip link set bat0 down
    fi

    # Add wlan0 interface to the batman-adv interface bat0
    sudo batctl if add wlan0

    # Set the Maximum Transmission Unit (MTU) of bat0 to 1468
    sudo ip link set dev bat0 mtu 1468

    # Pause for 1 second
    sleep 1

    # Set the bat0 interface up
    sudo ip link set bat0 up

    # Pause for 5 second
    sleep 5

    # Add an IP address (10.0.0.$IP) to the bat0 interface
    sudo ifconfig bat0 10.0.0.$IP/24

    # Print a message indicating that bat0 is configured
    echo "bat0 is configured."
}

# Check the input arguments
if [ $# -lt 1 ]; then
    echo "Please provide Drone number."
    exit 1
fi

# Calculate the IP address based on the input argument
IP=$(( $1 + 10 ))

# Print the Mesh IP and Wlan0 IP
echo "Mesh IP: 10.0.0.$IP"

# Check if the 'batctl' command is available
if command -v batctl &> /dev/null; then
    echo "The 'batctl' command is available."
else
    echo "The 'batctl' command is not available."
    echo "Now installing the 'batctl' command..."
    install_batctl
fi

# Check if the 'batman_adv' module is loaded
if lsmod | grep -q "batman_adv"; then
    echo "The 'batman_adv' module is loaded."
else
    echo "The 'batman_adv' module is not loaded."
    echo "Now loading the 'batman_adv' module..."
    load_batman_adv
fi

# Configure wlan0
config_wlan0

# Configure bat0
config_bat0