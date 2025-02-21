#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import pandas as pd
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class TopicLogger(Node):
    def __init__(self, duration=10):
        super().__init__('topic_logger')
        
        # Subscriptions
        self.subscription_pose = self.create_subscription(
            TransformStamped, '/mocap_pose', self.mocap_callback, 10)
        self.subscription_transform = self.create_subscription(
            TransformStamped, '/transform_topic', self.transform_callback, 10)
        
        # Data storage lists
        self.mocap_data = []
        self.transform_data = []

        self.duration = duration
        self.start_time = time.time()
        self.stop_logging = False

    def mocap_callback(self, msg):
        if time.time() - self.start_time < self.duration:
            self.mocap_data.append({            
                'timestamp': msg.header.stamp.sec ,#+ msg.header.stamp.nanosec * 1e-9,
                'translation_x': msg.transform.translation.x,
                'translation_y': msg.transform.translation.y,
                'translation_z': msg.transform.translation.z,
                'rotation_x': msg.transform.rotation.x,
                'rotation_y': msg.transform.rotation.y,
                'rotation_z': msg.transform.rotation.z,
                'rotation_w': msg.transform.rotation.w
            })
        elif not self.stop_logging:
            self.get_logger().info("Logging finished for /mocap_pose")
            self.stop_logging = True

    def transform_callback(self, msg):
        if time.time() - self.start_time < self.duration:
            self.transform_data.append({
                'timestamp': msg.header.stamp.sec , #+ msg.header.stamp.nanosec * 1e-9,
                'translation_x': msg.transform.translation.x,
                'translation_y': msg.transform.translation.y,
                'translation_z': msg.transform.translation.z,
                'rotation_x': msg.transform.rotation.x,
                'rotation_y': msg.transform.rotation.y,
                'rotation_z': msg.transform.rotation.z,
                'rotation_w': msg.transform.rotation.w
            })


def conv_csv_to_Tf(csv_df):
    ''' Convert CSV dataframe to transformation matrices '''
    samp, _ = csv_df.shape
    quat_x, quat_y, quat_z, quat_w = (
        csv_df['rotation_x'].values,
        csv_df['rotation_y'].values,
        csv_df['rotation_z'].values,
        csv_df['rotation_w'].values
    )
    tx, ty, tz = (
        csv_df['translation_x'].values,
        csv_df['translation_y'].values,
        csv_df['translation_z'].values
    )

    R_ = np.zeros((samp, 3, 3))
    _Tfs = np.zeros((samp, 4, 4))
    _Tfs[:, np.arange(4), np.arange(4)] = 1

    for i in range(samp):
        R_[i, :, :] = R.from_quat([quat_w[i], quat_x[i], quat_y[i], quat_z[i]]).as_matrix()
        _Tfs[i, 0:3, 0:3] = R_[i, :, :]
        _Tfs[i, 0:3, 3] = [tx[i], ty[i], tz[i]]

    return _Tfs, R_

def obtain_rot_cam_to_mocap(phi, theta):
    """Compute rotation matrix from camera frame to mocap frame."""
    rotation_y = np.array([
        [np.cos(theta),  0, -np.sin(theta)],
        [0,              1,  0],
        [np.sin(theta),  0,  np.cos(theta)]
    ])

    rotation_x = np.array([
        [1,  0,          0],
        [0,  np.cos(phi), np.sin(phi)],
        [0, -np.sin(phi), np.cos(phi)]
    ])

    return rotation_x @ rotation_y  # Apply Y rotation first, then X rotation

class TransformApplier(Node):
    def __init__(self, transform_matrix,tfm2):
        super().__init__('transform_applier')
        self.subscription = self.create_subscription(
            TransformStamped, '/transform_topic', self.callback, 10)
        self.publisher = self.create_publisher(TransformStamped, '/transformed_topic', 10)

        self.Tf_mocap_local_init_to_mocap = transform_matrix
        self.Tf_cam_to_mocap_local=tfm2

    def callback(self, msg):
        # Convert incoming message to transformation matrix
        t_in = np.array([msg.transform.translation.x, 
                         msg.transform.translation.y, 
                         msg.transform.translation.z, 1])

        r_in = R.from_quat([msg.transform.rotation.x,
                            msg.transform.rotation.y,
                            msg.transform.rotation.z,
                            msg.transform.rotation.w]).as_matrix()

        T_in = np.eye(4)
        T_in[0:3, 0:3] = r_in
        T_in[0:3, 3] = t_in[:3]

        # Apply transformation
        # T_out = self.Tf_mocap_local_to_mocap @ T_in
        # T_out=(self.Tf_mocap_local_init_to_mocap@self.Tf_cam_to_mocap_local@T_in.T@self.Tf_cam_to_mocap_local.T)
        T_out=(self.Tf_mocap_local_init_to_mocap@self.Tf_cam_to_mocap_local@np.linalg.inv(T_in)@self.Tf_cam_to_mocap_local.T)
        # T_out=np.linalg.inv(T_in)

        # print(T_out)
        # Extract transformed translation
        transformed_translation = T_out[0:3, 3]
        transformed_rotation = R.from_matrix(T_out[0:3, 0:3]).as_quat()

        # Publish transformed message
        transformed_msg = TransformStamped()
        transformed_msg.header.stamp = self.get_clock().now().to_msg()
        transformed_msg.header.frame_id = "world"
        transformed_msg.child_frame_id = "transformed_orb_in_mocap"

        transformed_msg.transform.translation.x = transformed_translation[0]
        transformed_msg.transform.translation.y = transformed_translation[1]
        transformed_msg.transform.translation.z = transformed_translation[2]

        transformed_msg.transform.rotation.x = transformed_rotation[0]
        transformed_msg.transform.rotation.y = transformed_rotation[1]
        transformed_msg.transform.rotation.z = transformed_rotation[2]
        transformed_msg.transform.rotation.w = transformed_rotation[3]

        self.publisher.publish(transformed_msg)
        self.get_logger().info("Published transformed data on /transformed_topic")


def main(args=None):
    rclpy.init(args=args)
    duration = 5  # Logging duration
    node = TopicLogger(duration)

    while time.time() - node.start_time < duration:
        rclpy.spin_once(node, timeout_sec=0.1)  

    # Convert collected data to Pandas DataFrames
    mocap_df = pd.DataFrame(node.mocap_data)
    transform_df = pd.DataFrame(node.transform_data)

    mocap_df.to_csv("mocap_csv_init.csv")
    transform_df.to_csv("transform_init.csv")

    node.get_logger().info(f"Collected Mocap Data: {len(mocap_df)} rows")
    node.get_logger().info(f"Collected Transform Data: {len(transform_df)} rows")

    # Extract initial transformation
    zero_condition = ((transform_df['translation_x'] == 0) & 
                      (transform_df['translation_y'] == 0) & 
                      (transform_df['translation_z'] == 0))

    ff_index = np.argmax(~zero_condition)
    timestamps = transform_df["timestamp"].values
    timestamp_i, timestamp_f = timestamps[0], timestamps[-1]

    print(f'time_i : {timestamp_i} ----> time_f : {timestamp_f}')

    mocap_data_at_0 = mocap_df[(mocap_df['timestamp'] >= timestamp_i) & 
                               (mocap_df['timestamp'] <= timestamp_f)]
    
    # Compute transformation matrix
    Init_Tfs, R_inits_ = conv_csv_to_Tf(mocap_data_at_0)
    Tf_mocap_local_to_mocap = np.mean(Init_Tfs, axis=0)

    print(f"Initial averaged transformation:\n {Tf_mocap_local_to_mocap}")


    # Add function to make transformation matrix to the mocap frame.
    # Compute the transformation matrix
    phi, theta = np.pi/2, 0
    Tf_cam_to_mocap_local = np.eye(4)
    Tf_cam_to_mocap_local[:3, :3] = np.round(obtain_rot_cam_to_mocap(phi, theta), 5)

    # Start the transformation applier
    _transform_applier = TransformApplier(Tf_mocap_local_to_mocap,Tf_cam_to_mocap_local)
    rclpy.spin(_transform_applier)  

    # Cleanup
    node.destroy_node()
    _transform_applier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
