import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
import pandas as pd
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class TopicLogger(Node):
    def __init__(self, duration=10):
        super().__init__('topic_logger')
        
        # Subscriptions
        self.subscription_pose = self.create_subscription(
            PoseStamped, '/mocap_pose', self.mocap_callback, 10)
        self.subscription_transform = self.create_subscription(
            TransformStamped, '/transform_topic', self.transform_callback, 10)
        
        # Data storage lists
        self.mocap_data = []
        self.transform_data = []

        self.duration = duration
        self.start_time = time.time()

    def mocap_callback(self, msg):
        if time.time() - self.start_time < self.duration:
            self.mocap_data.append({
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'position_x': msg.pose.position.x,
                'position_y': msg.pose.position.y,
                'position_z': msg.pose.position.z,
                'orientation_x': msg.pose.orientation.x,
                'orientation_y': msg.pose.orientation.y,
                'orientation_z': msg.pose.orientation.z,
                'orientation_w': msg.pose.orientation.w
            })
        else:
            self.get_logger().info("Logging finished for /mocap_pose")

    def transform_callback(self, msg):
        if time.time() - self.start_time < self.duration:
            self.transform_data.append({
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'translation_x': msg.transform.translation.x,
                'translation_y': msg.transform.translation.y,
                'translation_z': msg.transform.translation.z,
                'rotation_x': msg.transform.rotation.x,
                'rotation_y': msg.transform.rotation.y,
                'rotation_z': msg.transform.rotation.z,
                'rotation_w': msg.transform.rotation.w
            })
        else:
            self.get_logger().info("Logging finished for /transform_topic")
#--------- Aux functions for transformations -------------

# ----------- Converting the csv into transformation matrices ----------
def conv_csv_to_Tf(csv_df):
    ''' 
    Assuming the keys to be same. Above keys are followed here since they are logged separately.

    '''
    samp,column=csv_df.shape

    quat_x,quat_y,quat_z,quat_w=csv_df['rotation_x'].values,csv_df['rotation_y'].values,csv_df['rotation_z'].values,csv_df['rotation_z'].values

    tx,ty,tz=csv_df['translation_x'].values,csv_df['translation_y'].values,csv_df['translation_z'].values

    R_=np.zeros((samp,3,3))

    _Tfs=np.zeros((samp,4,4))
    _Tfs[:,np.arange(4), np.arange(4)]=1
    for  i in range(0,samp):
        # Define quaternion (w, x, y, z)
    # Convert to rotation matrix
        R_[i,:,:]=R.from_quat([quat_x[i],quat_y[i],quat_z[i],quat_w[i]]).as_matrix()
        _Tfs[i,0:3,0:3]=R_[i,:,:]
        _Tfs[i,0:3,3]=[tx[i],ty[i],tz[i]]
        

    return _Tfs,R_


def main(args=None):
    rclpy.init(args=args)
    duration = 5  # Set logging duration in seconds
    node = TopicLogger(duration)

    # Run the node for 'duration' seconds
    rclpy.spin_once(node, timeout_sec=duration)

    # Convert data to Pandas DataFrames
    mocap_df = pd.DataFrame(node.mocap_data)
    transform_df = pd.DataFrame(node.transform_data)

    # Matching the initial origin of orbslam zeros
    zero_condition=((transform_df['translation_x']==0).values & (transform_df['translation_y']==0).values & (transform_df['translation_z']==0).values)

    ff_index=np.argmax(~zero_condition) # Index where orbslam has started giving some data

    timestamp_i=transform_df["timestamp"][0]
    timestamp_f=transform_df["timestamp"][ff_index-1]

    print(f'time_i : {timestamp_i} ----> time_f : {timestamp_f}')

    mocap_data_at_0 = mocap_df[(mocap_df['timestamp'] >= timestamp_i) & (mocap_df['timestamp'] <= timestamp_f)]
    
    # Converting the dfs to Tfs
    Init_Tfs,R_inits_=conv_csv_to_Tf(mocap_data_at_0)

    # Computation of initial transformation
    ''' ---------Initial Transformation of mocap local to mocap frame is --------'''
    
    Tf_mocap_local_to_mocap=np.mean(Init_Tfs,axis=0)
    
    print(f"Initial averaged transformation is\n {Tf_mocap_local_to_mocap}")


    # # Print first few rows
    # print("Mocap Pose Data:\n", mocap_df.head())
    # print("Transform Data:\n", transform_df.head())

    # Destroy node and shut down
    node.destroy_node()
    rclpy.shutdown()

    return mocap_df, transform_df  # Return DataFrames as variables

if __name__ == '__main__':
    mocap_df, transform_df = main()
