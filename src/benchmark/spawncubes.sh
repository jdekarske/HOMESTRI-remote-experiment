#!/bin/bash

rosservice call /spawn_objects_service "{param_name: '/cube_positions/inputs', \
    overwrite: true, position: 0, color: [0,0,1], length: 0, width: 0}"

for ((i = 1; i < 5; i++)); do
    rosservice call /pick_place "{pick_object: 'cube_$i', place_object: 'cube_$i'}"
done

