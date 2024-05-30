roslaunch random_map_generator map_server.launch & sleep 1;
roslaunch kinematics_simulator mesh_vis.launch & sleep 1;
roslaunch plan_manage plan_node.launch & sleep 1;
roslaunch plan_manage test_mpc.launch  & sleep 1;
# rosbag play 2023-01-29-20-04-36.bag
wait

