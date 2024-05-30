# roslaunch random_map_generator test.launch & sleep 1; #not sending map now
roslaunch plan_manage plan_node.launch & sleep 1;
# roslaunch plan_manage test_mpc.launch  & sleep 1;
# rosbag play 2023-01-29-20-04-36.bag
wait

