rosservice call /spawn_objects_service "{param_name: '/cube_positions/inputs', overwrite: true, length: 0, width: 0}"

rosservice call /pick_place "{pick_object: 'cube_1', place_object: 'cube_1'}"
rosservice call /pick_place "{pick_object: 'cube_2', place_object: 'cube_4'}"
rosservice call /pick_place "{pick_object: 'cube_3', place_object: 'cube_5'}"
rosservice call /pick_place "{pick_object: 'cube_4', place_object: 'cube_8'}"