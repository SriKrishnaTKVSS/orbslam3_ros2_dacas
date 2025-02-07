import numpy as np
import pandas as pd
from decimal import Decimal
import os
import sympy as sp
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import time

def obtain_rot_cam_to_mocap():
    phi,theta=sp.symbols('phi theta')
    rotation_y = sp.Matrix([
            [sp.cos(theta), 0, -sp.sin(theta)],
            [0, 1, 0],
            [sp.sin(theta), 0, sp.cos(theta)]
        ])

    # Next, rotate around the new x-axis by 90 degrees
    rotation_x = sp.Matrix([
        [1, 0, 0],
        [0, sp.cos(phi), sp.sin(phi)],
        [0, -sp.sin(phi), sp.cos(phi)]
    ])

    # Combined rotation: First y-rotation, then x-rotation
    rotation = rotation_x @ rotation_y  # Apply rotation_y first, then rotation_x
    return sp.lambdify((phi, theta), rotation)
def process_mocap_file(filename):

    data = []
    

    ## ---------- Name for the new file from given filename ---------
    # Creating a new processed file
    pos = filename.rfind('.')
    new_file = filename[:pos] + "_processed.txt"
    # print(new_file)

    # Remove the file if it already exists
    if os.path.exists(new_file):
        os.remove(new_file)


    # Open the new file to write processed data
    with open(new_file, 'a') as f:
        try:
            with open(filename, 'r') as file:
                count_lines = 0

                # Read each line from the file
                for line in file:
                    row = []
                    has_nan = False
                    values = line.split()

                    # Process each value in the row
                    for value in values:
                        try:
                            num = float(value)  # Convert to float
                            if num == num:  # Check if it's not NaN
                                row.append(num)
                            else:
                                has_nan = True
                        except ValueError as e:
                            # If conversion fails (e.g., if the value is 'NaN'), skip it
                            # has_nan = True
                            print(f"Error opening file: {filename}. Exception: {str(e)}")
                            # break
                    
                    count_lines += 1

                    # Add the row to data if it doesn't have NaN
                    if not has_nan:
                        data.append(row)

                print(f"The total number of remaining lines in {filename} is {count_lines}")

        except Exception as e:
            print(f"Error opening file: {filename}. Exception: {str(e)}")

        # Write the filtered data to the new file
        print("Filtered data (no NaN values):")
        for row in data:
            for num in row:
                # print(f"{num:.6f}", end=" ")  # Print with fixed decimal format
                f.write(f"{num:.6f} ")  # Write to the file
            # print()
            f.write("\n")

        # f.write("2\n")

    return new_file
def transformToMocap(Tf_local,Tf_to_mocap,matched_rows_orbslam):
    num_matrices = len(matched_rows_orbslam.iloc[:,0]) 
    matrices_array = np.zeros((num_matrices, 4, 4))
    for i in range(0,num_matrices):
        quat=matched_rows_orbslam.iloc[i,4:]
        tf=np.eye(4)
        tf[:3,:3]=R.from_quat(quat).as_matrix() #scalar last
        tf[:3,3]=matched_rows_orbslam.iloc[i,1:4]
        matrices_array[i]=(Tf_to_mocap@Tf_local@tf@Tf_local.T)
    return matrices_array

def rot2euler(x):
    n=len(x)
    eul=np.zeros((n,3,))
    for i in range(0,n):
        eul[i,:]= (R.from_matrix(x[i])).as_euler('xyz',degrees=True)
    return eul

def writing_to_txt(s,data):
    os.makedirs(os.path.dirname(s), exist_ok=True)
    with open(s, "w") as file:
    # Transpose the columns to rows using zip
        for row in zip(*data.values()):  # Combine values row by row
            file.write(" ".join(map(str, row)) + "\n")


def main():

    filename_mocap = "/home/srikrishna/Desktop/Projects/scale_est/Krishna_pybind_orsbslam3/src/ORB_SLAM3/Mocap_data.txt"  # Replace with your file path
    mocap_processed = process_mocap_file(filename_mocap)
    print(f"Processed file: {mocap_processed}")

    data_mocap_processed=pd.read_csv('Mocap_data_processed.txt',header=None,delimiter=' ')
    data_mocap_processed=data_mocap_processed.applymap(Decimal)


        # ------------------ Orbslam Camera Trajectory ----------------------
    orbslam_data=pd.read_csv('/home/srikrishna/Desktop/Projects/scale_est/Krishna_pybind_orsbslam3/src/ORB_SLAM3/CameraTrajectory.txt',header=None,delimiter=' ')
    orbslam_data=orbslam_data.applymap(Decimal)

    print(len(orbslam_data.iloc[:,0]))

    matched_rows_mocap = data_mocap_processed[data_mocap_processed.iloc[:,0].isin(orbslam_data.iloc[:,0])]
    matched_rows_orbslam = orbslam_data[orbslam_data.iloc[:,0].isin(data_mocap_processed.iloc[:,0])]

    ##------------
    # Add function to make transformation matrix to the mocap frame.
    rot_cam_to_mocap_local=obtain_rot_cam_to_mocap() # 

    # ---- Mocap data
    tim_mocap=(matched_rows_mocap.iloc[:,0]-matched_rows_mocap.iloc[0,0])#/(1e6)
    tim_mocap=np.array([int(d)/(1e9) for d in tim_mocap])
    pos_mocap=np.zeros((len(matched_rows_mocap.iloc[:,0]), 3,))
    pos_mocap[:,0]= matched_rows_mocap.iloc[:,1]
    pos_mocap[:,1]= matched_rows_mocap.iloc[:,2]
    pos_mocap[:,2]= matched_rows_mocap.iloc[:,3]+1200
    rot_mats_mocap=np.zeros((len(matched_rows_mocap.iloc[:,0]), 3,3))
    # rot_mats_mocap[:,0,:3]=matched_rows_mocap.iloc[:,4:7]
    # rot_mats_mocap[:,1,:3]=matched_rows_mocap.iloc[:,7:10]
    # rot_mats_mocap[:,2,:3]=matched_rows_mocap.iloc[:,10:13]
    rot_mats_mocap[:,:3,0]=matched_rows_mocap.iloc[:,4:7]
    rot_mats_mocap[:,:3,1]=matched_rows_mocap.iloc[:,7:10]
    rot_mats_mocap[:,:3,2]=matched_rows_mocap.iloc[:,10:13]


    #---- Initial transformation -----
    Tf_cam_to_mocap_local=np.eye(4)
    Tf_cam_to_mocap_local[:3,:3]=np.round(rot_cam_to_mocap_local(np.pi/2,0),5)
    Tf_mocap_local_to_mocap=np.eye(4)
    Tf_mocap_local_to_mocap[:3,:3]=rot_mats_mocap[0]
    Tf_mocap_local_to_mocap[:3,3]=np.array(matched_rows_mocap.iloc[0,1:4]/1000) # converting to meters as mocap gives in mm


    # transform everything from orbslam to mocap
    orb_in_mocap= transformToMocap(Tf_cam_to_mocap_local,Tf_mocap_local_to_mocap,matched_rows_orbslam=matched_rows_orbslam)
    
    ## Extracting and Plotting
    # ---- Orb data
    tim_orb=matched_rows_orbslam.iloc[:,0]-matched_rows_orbslam.iloc[0,0]
    tim_orb=np.array([int(d)/(1e9) for d in tim_orb])
    pos_orb=np.zeros((len(orb_in_mocap), 3,))
    pos_orb=orb_in_mocap[:,:3,3]*1000
    rot_mats_orb=orb_in_mocap[:,:3,:3]



    # Obtaing euler angles from rot mats
    euler_orbs=rot2euler(rot_mats_orb)
    euler_mocap=rot2euler(rot_mats_mocap)
    print(abs(euler_mocap[:,0])-abs(euler_orbs[:,0]))

    # Saving the modified files.
    # orbslam_in_mocap={}
    orbslam_in_mocap_data={'timestamp':matched_rows_orbslam.iloc[:,0],'x':pos_orb[:,0],'y':pos_orb[:,1],'z':pos_orb[:,2],'roll':euler_orbs[:,0],'pitch':euler_orbs[:,1],'yaw':euler_orbs[:,2]}
    mocap_in_mocap_data_dprocessed={'timestamp':matched_rows_mocap.iloc[:,0],'x':pos_mocap[:,0],'y':pos_mocap[:,1],'z':pos_mocap[:,2],'roll':euler_mocap[:,0],'pitch':euler_mocap[:,1],'yaw':euler_mocap[:,2]}
    
    s_orb='/home/srikrishna/Desktop/Projects/scale_est/Krishna_pybind_orsbslam3/src/ORB_SLAM3/orbslam_data_in_mocap.txt'
    s_mocap='/home/srikrishna/Desktop/Projects/scale_est/Krishna_pybind_orsbslam3/src/ORB_SLAM3/mocap_data_in_mocap.txt'
    # Writing them to text files
    writing_to_txt(s_orb,orbslam_in_mocap_data)
    writing_to_txt(s_mocap,mocap_in_mocap_data_dprocessed)
    
    # plotting
    plt.figure(1)
    title='Comparison of positions and orientations between orbslam3 and mocap data in mocap frame'
    plt.title(title,loc='center',wrap=True)
    plt.subplot(6,1,1)
    plt.plot(tim_mocap,pos_mocap[:,0],'r-',tim_orb,pos_orb[:,0],'b--')
    plt.legend(['mocap','orbslam'])
    plt.xlabel('time(s)')
    plt.ylabel('x(mm)')
    plt.grid()
    
    plt.subplot(6,1,2)
    plt.plot(tim_mocap,pos_mocap[:,1],'r-',tim_orb,pos_orb[:,1],'b--')
    plt.legend(['mocap','orbslam'])
    plt.xlabel('time(s)')
    plt.ylabel('y(mm)')
    plt.grid()

    plt.subplot(6,1,3)
    plt.plot(tim_mocap,pos_mocap[:,2],'r-',tim_orb,pos_orb[:,2]+1200,'b--')
    plt.legend(['mocap','orbslam'])
    plt.xlabel('time(s)')
    plt.ylabel('z(mm)')
    plt.grid()

    plt.subplot(6,1,4)
    plt.plot(tim_mocap,euler_mocap[:,0],'r-',tim_orb,euler_orbs[:,0],'b--')#-,+90
    plt.legend(['mocap','orbslam'])
    plt.xlabel('time(s)')
    plt.ylabel(r'${\phi}(degrees)$')
    plt.grid()
    
    plt.subplot(6,1,5)
    plt.plot(tim_mocap,euler_mocap[:,1],'r-',tim_orb,euler_orbs[:,1],'b--')
    plt.legend(['mocap','orbslam'])
    plt.xlabel('time(s)')
    plt.ylabel(r'${\theta}(degrees)$')
    plt.grid()


    plt.subplot(6,1,6)
    plt.plot(tim_mocap,euler_mocap[:,2],'r-',tim_orb,euler_orbs[:,2],'b--')#+90
    plt.legend(['mocap','orbslam'])
    plt.xlabel('time(s)')
    plt.ylabel(r'${\psi}(degrees)$')
    # plt.ylabel(r'${\psi}^{\circ}$')
    plt.grid()
    
    print("Saving the figure....")
    # Save the figure as a PDF file (vector format)
    plt.savefig('figure.pdf', format='pdf')
    # print("Waiting for 10 seconds....")
    # time.sleep(10)    
    # plt.show()

if __name__ == "__main__":
    main()



# print(init_timestamp)


