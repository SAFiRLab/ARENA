#!/bin/bash


sleep 1;
gnome-terminal -- rosrun linedrone_navigation costmap_3D_node
rosparam load $(rospack find linedrone_navigation)/config/costmap_3d_node.yaml

rosparam set /data/world_name "A386"

sleep 5;
gnome-terminal -- bash -c "roslaunch linedrone_navigation testbench.launch"

sleep 2;
gnome-terminal -- bash -c "roslaunch linedrone_navigation pagmo_optimizer.launch is_testing:=true"

param_names=(
  "/pagmo_optimizer_node/voting_algorithm/cost_time"
  "/pagmo_optimizer_node/voting_algorithm/cost_safety"
  "/pagmo_optimizer_node/voting_algorithm/cost_energy"
)


# Define the precision
precision=1.0
max_value=1.0

# Calculate the number of steps based on the precision
num_steps=$(bc <<< "scale=0; ($max_value / $precision) + 1")
#num_steps=10

path_planning_finished_counter=0

# Iterate over the parameter combinations
for ((i1 = 0; i1 < num_steps; i1++)); do
  for ((i2 = 0; i2 < num_steps; i2++)); do
    for ((i3 = 0; i3 < num_steps; i3++)); do
      # Calculate the parameter values
      value1=$(bc <<< "$i1 * $precision")
      value2=$(bc <<< "$i2 * $precision")
      value3=$(bc <<< "$i3 * $precision")

      # Set the parameter values

      rosparam set ${param_names[0]} $value1 &

      rosparam set ${param_names[1]} $value2 &

      rosparam set ${param_names[2]} $value3 &

      # Display the parameter values
      echo "Parameters set to [$value1,$value2,$value3]"

      rostopic pub -1 /navigation/3d/planning_activated std_msgs/Bool "data: true"

      sleep 0.2;
      rostopic pub -1 /navigation/3d/goal geometry_msgs/PointStamped "{
        header: {
          seq: 0,
          stamp: { secs: 0, nsecs: 0 },
          frame_id: 'map'
        },
        point: {
          x: 30.0,
          y: 20.0,
          z: 35.0
        }
      }"

      # Wait for path planning to finish
      while :
      do
        counter=$(rostopic echo -n 1 /testbench/path_planning_finished_counter | grep -oP '(?<=data: )\w+')
        echo "Counter: $counter"

        if (( $counter > path_planning_finished_counter )); then
          echo "Path planning finished"
          path_planning_finished_counter=$counter
          break
        fi
      done
    done
  done
done
