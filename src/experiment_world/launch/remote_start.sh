
# Launch script in background
roslaunch experiment_world main_experiment.launch rviz_gui:=false gazebo_gui:=false &

# Get its PID
PID=$!

# Wait for it to start
sleep 60

# Kill it
kill $PID

# wait for it to die
sleep 5

roslaunch experiment_world main_experiment.launch rviz_gui:=false gazebo_gui:=false &