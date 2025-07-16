How to run the code:

1.First we need to download the code in the appropiate repository
- cd ~/ws_moveit/src
- git clone https://github.com/Starajo/IndustrialRos.git
- cd ~/ws_moveit
- catkin build
- source ~/ws_moveit/devel/setup.bash

2.Then, we can run the code as it follows:
- roslaunch panda_moveit_config demo_gazibo.launch
- rosrun prueba1_linea lissajous_node



The organization for the project files is the following one:

- cd cd ~/ws_moveit/src/prueba1_linea
- ls
  CMakeLists.txt  package.xml  scripts  src
-  cd src
-  ls
  
  ee_pose.csv;               ideal.csv;                     trajectory.csv;
  extract_tf_trajectory.py;  lissajous_data.bag;            trayectoria_ideal.csv;
  frames.gv;                 lissajous_node.cpp;
  frames.pdf;                plot_lissajous_comparison.py;



Important files:

1. lissajous_node.cpp:
  =>Initializes ROS, the MoveIt planning interface (MoveGroupInterface) for the "panda_arm" group, and the visualization tools for RViz.
  =>Generates a set of Cartesian waypoints following a Lissajous curve based on configurable amplitude and frequency parameters (A, B, a, b, delta).
  =>Publishes the ideal trajectory as a PoseArray on the /ideal_lissajous_path topic. Saves the generated trajectory as a .csv file (trayectoria_ideal.csv).
  =>Compute a Cartesian trajectory from the waypoints using computeCartesianPath().
2. extract_tf_trajectory.py: This script records the end-effector trajectory of the Franka Emika Panda robot by listening to TF transforms during execution, and compares it with an ideal path.
  =>Subscribes to TF and continuously looks up the transform between the base (panda_link0) and the end-effector (panda_link8).
  =>Stores the real trajectory data (x_data, y_data, z_data) as the robot moves and saves the trajectory to a CSV file called trajectory.csv.
