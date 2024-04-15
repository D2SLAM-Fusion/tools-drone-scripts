#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

function sysbench_cpu() {
    if command -v sysbench &> /dev/null; then
        echo "The 'sysbench' command is available."
    else
        echo "The 'sysbench' command is not available."
        echo "Now installing the 'sysbench' command..."
        sudo apt-get install -y sysbench
    fi
    echo "Running sysbench cpu benchmark"
    sysbench cpu --max-time=180 --threads=$(nproc) run
}

function trt_workload() {
    if [ -f "/usr/src/tensorrt/bin/trtexec" ]; then
        echo "The 'trtexec' command is available."
    else
        echo "The 'trtexec' command is not available."
        echo "Please install TensorRT first."
        exit 1
    fi

    echo "Running trtexec workload"
    /usr/src/tensorrt/bin/trtexec --onnx=$SCRIPT_DIR/trt/hitnet.onnx --saveEngine=$SCRIPT_DIR/trt/hitnet.trt --best
    echo "Finished generating TensorRT engine"
    /usr/src/tensorrt/bin/trtexec --onnx=$SCRIPT_DIR/trt/hitnet.onnx --loadEngine=$SCRIPT_DIR/trt/hitnet.onnx --int8 --fp16 --iterations=1000 --streams=4 --useSpinWait
    echo "Finished running TensorRT workload"
}

function main() {
    sudo nvpmodel -m 0
    { sysbench_cpu; } & pid1=$!
    { trt_workload; } & pid2=$!

    wait $pid1
    wait $pid2
    echo "Finished running all benchmarks"
}

main