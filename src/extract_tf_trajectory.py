#!/usr/bin/env python  
import rospy
import tf2_ros
import matplotlib.pyplot as plt
import signal
import sys
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

# Global variables to store the data
x_data = []
y_data = []
z_data = []

def signal_handler(sig, frame):
    rospy.loginfo("Shutting down node, saving CSV and plotting data")

    # Save data to CSV
    if x_data and y_data and z_data:
        df = pd.DataFrame({
            'x': x_data,
            'y': y_data,
            'z': z_data
        })
        df.to_csv('trajectory.csv', index=False)
        rospy.loginfo("Saved trajectory to trajectory.csv")

        try:
            ideal_df = pd.read_csv('trayectoria_ideal.csv')
            ideal_x = ideal_df['x']
            ideal_y = ideal_df['y']
            ideal_z = ideal_df['z']
        except Exception as e:
            rospy.logwarn(f"Could not read ideal trajectory CSV: {e}")
            ideal_x, ideal_y, ideal_z = [], [], []

        # Plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_data, y_data, z_data, label='Recorded Trajectory', color = 'blue')
        if ideal_x and ideal_y and ideal_z:
            ax.plot(ideal_x, ideal_y, ideal_z, label='Ideal Trajectory', color='red', linestyle='--')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Trajectory of panda_hand_tcp relative to panda_link0')
        plt.show()

    sys.exit(0)

def main():
    rospy.init_node('tf2_listener_example')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)

    signal.signal(signal.SIGINT, signal_handler)

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform('panda_link0', 'panda_link8', rospy.Time(0))
            t = trans.transform.translation
            r = trans.transform.rotation

            rospy.loginfo("Translation: [%.3f, %.3f, %.3f]" % (t.x, t.y, t.z))

            x_data.append(t.x)
            y_data.append(t.y)
            z_data.append(t.z)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not available yet.")
            pass

        rate.sleep()

if __name__ == '__main__':
    main()
