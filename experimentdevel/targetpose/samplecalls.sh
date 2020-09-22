#!/bin/bash
rosservice call /pick_place "pick_object: 'cube_1'
place_object: 'cube_0'"

rosservice call /pick_place "pick_object: 'cube_2'
place_object: 'cube_1'"

rosservice call /pick_place "pick_object: 'cube_3'
place_object: 'cube_2'"