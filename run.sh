#!/bin/bash

pids=()

# miniros server --superserver ./cfg/superserver.config.json & pids+=($!)
miniros server > logs/server.log & 
pids+=($!)

miniros run vlidar > logs/vlidar.log & 
pids+=($!)

miniros run vslam > logs/vslam.log &
pids+=($!)

miniros run vpathfinder > logs/vpathfinder.log &
pids+=($!)

miniros run vmovement > logs/vmovement.log &
pids+=($!)


# Function to check if background processes are still alive
check_background_jobs() {
    for pid in "${pids[@]}"; do
        if ! ps -p "$pid" > /dev/null; then
            echo "ERROR: Process $pid exited early"
            exit 1
        fi
    done
}

# Check initial process status
check_background_jobs

miniros run vmain
echo OK 

wait

kill "${pids[@]}" 2>/dev/null
wait 2>/dev/null