#!/bin/bash
ssty -echoctl

pids=()

# miniros server --superserver ./cfg/superserver.config.json & pids+=($!)
# miniros server > logs/server.log & 
# pids+=($!)

miniros run vlidar > logs/vlidar.log & 
pids+=($!)

miniros run vslam > logs/vslam.log &
pids+=($!)

miniros run vpathfinder > logs/vpathfinder.log &
pids+=($!)

miniros run vmovement > logs/vmovement.log &
pids+=($!)

miniros run vmain > logs/vmain.log &
pids+=($!)

# check if background processes are still alive
check_background_jobs() {
    for pid in "${pids[@]}"; do
        if ! ps -p "$pid" > /dev/null; then
            echo "One of processes stopped - exiting"
            return false
        fi
    done

    return true
}

exit_prog() {
    echo Killing processes...
    kill "${pids[@]}" 2>/dev/null
    wait 2>/dev/null
    echo Done
}

trap 'exit_prog' INT
echo Started all scripts

while check_background_jobs(); do
    sleep 1s
done

exit_prog()