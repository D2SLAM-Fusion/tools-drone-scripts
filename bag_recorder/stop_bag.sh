#!/usr/bin/env bash
source ./bag_config.sh
echo "Stop bag record"
while IFS='' read -r line || [[ -n "$line" ]]; do
    IFS=':' inarr=(${line})
    if [ ${inarr[0]} = "rosbag" ]; then
        echo "Killing $line with ${inarr[1]}"
        echo dji | sudo -S kill -INT ${inarr[1]}
    fi
done < $BAG_PID_FILE