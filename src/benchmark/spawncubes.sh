#!/bin/bash

num_runs=1
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -n|--num_runs) num_runs=$2; shift ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

for ((i = 0; i < $num_runs; i++)); do
    # reset the arm
    rosservice call /pick_place/reset "{}"

    # delete all cubes
    rosservice call /spawn_objects_service "{param_name: '', \
        overwrite: true, position: 0, color: [0], length: 0, width: 0}"

    # spawn cubes
    for ((j = 1; j < 5; j++)); do
    rosservice call /spawn_objects_service "{param_name: '/cube_positions/inputs', \
        overwrite: false, position: $j, color: [0,0,1], length: 0, width: 0}"
    done

    # pickplace cunes
    for ((k = 1; k < 5; k++)); do
        rosservice call /pick_place "{pick_object: 'cube_$k', place_object: 'cube_$k'}"
    rosservice call /pick_place/reset "{}" 
    done
done
