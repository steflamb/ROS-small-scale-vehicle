import math
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from hampel import hampel
import numpy as np
import os
import tf



SIZE_FACTOR=7.33
X_MAP_SHIFT=48
Y_MAP_SHIFT=50
    

def rotate(p, degrees=0):
    angle = np.deg2rad(degrees)
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    p = np.atleast_2d(p)

    return p @ R.T

"""
The path format is
path_prefix ## SPEED ## ITERATION ## path_suffix
"""
path_prefix = "actuation/forward_difference/t"
path_suffix = ".bag"

min_throttle = 0.32
max_throttle = 0.39
throttle_step = 0.01

for throttle in [0.39]:
    directory = "actuation/forward_difference"
    bags = [fn for fn in os.listdir(directory)
               if ".bag" in fn]
    matches = [fn for fn in bags if "%0.2f"%throttle in fn]
    number_of_iterations = len(matches)
    print(f"{number_of_iterations} iterations of throttle {'%0.2f'%throttle} found")

    fig, axs = plt.subplots(2, 1)
    fig.suptitle("Forward difference throttle " + str(throttle))

    # for iteration in range(1,number_of_iterations+1):
    for iteration in [1]:
        path = path_prefix + "%0.2f"%throttle + "_" + str(iteration) + path_suffix
        print(f"Path is {path}")
        b = bagreader(path)

        frames = {}
        for t in b.topics:
            data = b.message_by_topic(t)
            frames[t] = pd.read_csv(data).set_index("Time")

        control = frames["/control/throttle_steering"]

        for t in b.topics:
            if t!="/control/throttle_steering":
                #crop data to only when a command is given
                frames[t] = frames[t][control.first_valid_index():control.last_valid_index()]
                #shift index to start time at 0 (when first control command was sent)
                new_index = list(map(lambda x: x-control.first_valid_index(), list(frames[t].index.values)))
                frames[t].reindex(new_index)

                if iteration ==1:
                    print(t)
                    print(frames[t].columns)
                    print(frames[t])
        


        #ADJUST FRAME OF REFERENCE
        #change units of mixed to real
        frames["/sim/speed"] = frames["/sim/speed"].apply(lambda x: x/SIZE_FACTOR)
        
        #shift all positions so the start is at 0,0
        frames["/donkey/pose"]["pose.position.x"] = frames["/donkey/pose"]["pose.position.x"].apply(lambda x: x-frames["/donkey/pose"]["pose.position.x"][frames["/donkey/pose"].first_valid_index()])
        frames["/donkey/pose"]["pose.position.y"] = frames["/donkey/pose"]["pose.position.y"].apply(lambda x: x-frames["/donkey/pose"]["pose.position.y"][frames["/donkey/pose"].first_valid_index()])

        frames["/sim/euler"]["x"] = frames["/sim/euler"]["x"].apply(lambda x: (x-X_MAP_SHIFT)/SIZE_FACTOR - frames["/sim/euler"]["x"].first_valid_index())
        frames["/sim/euler"]["z"] = frames["/sim/euler"]["z"].apply(lambda x: (x-Y_MAP_SHIFT)/SIZE_FACTOR - frames["/sim/euler"]["z"].first_valid_index())
        

        #rotate all points to have initial angle = 0
        #DONKEY POSE
        #convert quat to deg
        quaternion = [frames["/donkey/pose"].loc[:,["pose.orientation.x"]].iloc[0][0],
                      frames["/donkey/pose"].loc[:,["pose.orientation.y"]].iloc[0][0],
                      frames["/donkey/pose"].loc[:,["pose.orientation.z"]].iloc[0][0],
                      frames["/donkey/pose"].loc[:,["pose.orientation.w"]].iloc[0][0]]
        rad = tf.transformations.euler_from_quaternion(quaternion)[2]
        angle = math.degrees(rad)
        #rotate
        points = frames["/donkey/pose"].loc[:,["pose.position.x", "pose.position.y"]].to_numpy()
        mixed_real = rotate(points, degrees=-angle)
        #put back in table
        frames["/donkey/pose"].loc[:,["pose.position.x"]] = pd.Series(mixed_real.T[0], index = frames["/donkey/pose"].index)
        frames["/donkey/pose"].loc[:,["pose.position.y"]] = pd.Series(mixed_real.T[1], index = frames["/donkey/pose"].index)  
        pass

        #MIXED POSE
        #get point pairs
        points = frames["/sim/euler"].loc[:,["x","z"]].to_numpy()
        mixed_rotated = rotate(points, degrees=-frames["/sim/euler"]["yaw"][frames["/sim/euler"].first_valid_index()])
        # print(mixed_rotated)
        #put back in table   
        frames["/sim/euler"].loc[:,["x"]] = pd.Series(mixed_rotated[0].T[0], index = frames["/sim/euler"].index)
        frames["/sim/euler"].loc[:,["z"]] = pd.Series(mixed_rotated[0].T[1], index = frames["/sim/euler"].index)  
        

        


        # PLOT
        #position (path) comparison
        # if iteration==1:
        axs[0].set_title("x over time")
        axs[0].plot(frames["/donkey/pose"].index, frames["/donkey/pose"]["pose.position.x"], color="lightskyblue")
        # axs[0].plot(frames["/sim/euler"].index, frames["/sim/euler"]["x"], color="darkorange")

        axs[1].set_title("y over time")
        axs[1].plot(frames["/donkey/pose"].index, frames["/donkey/pose"]["pose.position.y"], color="lightskyblue")
        # axs[1].plot(frames["/sim/euler"].index, frames["/sim/euler"]["z"], color="darkorange")
        # axs[iteration-1,0].set_xlim([-2,2])
        # axs[iteration-1,0].set_ylim([-2,2])

        #speed comparison
        """
        hampel_donkey_speed = hampel(frames["/donkey/speed"]["data"], window_size=10)
        outliers = frames["/donkey/speed"].iloc[hampel_donkey_speed.outlier_indices]
        #UNCOMMENT TO SHOW OUTLIERS
        axs[iteration-1,1].plot(frames["/donkey/speed"], color="lightskyblue")
        axs[iteration-1,1].scatter(outliers.index, outliers["data"].values, color="red")
        
        filtered_speed = frames["/donkey/speed"].drop(outliers.index)
        savgol_donkey_speed = savgol_filter(filtered_speed["data"], 50, 3)
        axs[iteration-1,1].plot(filtered_speed, color="dodgerblue")
        axs[iteration-1,1].plot(filtered_speed.index, savgol_donkey_speed, color="blue")

        hampel_speed = hampel(frames["/sim/speed"]["data"], window_size=10)
        outliers = frames["/sim/speed"].iloc[hampel_speed.outlier_indices]
        filtered_sim_speed = frames["/sim/speed"].drop(outliers.index)
        savgol_speed = savgol_filter(filtered_sim_speed["data"], 50, 3)
        axs[iteration-1,1].plot(frames["/sim/speed"], color ="orange")
        axs[iteration-1,1].plot(filtered_sim_speed.index, savgol_speed, color="darkorange")

        #acceleration comparison
        filtered_speed["time"] = pd.Series(filtered_speed.index,index = filtered_speed.index)
        filtered_speed = filtered_speed.diff()
        filtered_speed.drop(filtered_speed.first_valid_index(),inplace=True)
        filtered_speed["accel"] = filtered_speed["data"]/filtered_speed["time"]
        
        hampel_accel = hampel(filtered_speed["accel"], window_size=10)
        outliers = filtered_speed.iloc[hampel_accel.outlier_indices]
        #UNCOMMENT TO SHOW OUTLIERS
        axs[iteration-1,2].plot(filtered_speed["accel"], color="lightskyblue")
        axs[iteration-1,2].scatter(outliers.index, outliers["accel"].values, color="red")
        
        filtered_donkey_accel = filtered_speed.drop(outliers.index)
        savgol_accel = savgol_filter(filtered_donkey_accel["accel"], 50, 3)
        axs[iteration-1,2].plot(filtered_donkey_accel["accel"], color="dodgerblue")
        axs[iteration-1,2].plot(filtered_donkey_accel.index, savgol_accel, color="blue")


        frames["/sim/speed"]["time"] = pd.Series(frames["/sim/speed"].index,index = frames["/sim/speed"].index)
        frames["/sim/speed"] = frames["/sim/speed"].diff()
        frames["/sim/speed"].drop(frames["/sim/speed"].first_valid_index(),inplace=True)
        frames["/sim/speed"]["accel"] = frames["/sim/speed"]["data"]/frames["/sim/speed"]["time"]

        hampel_accel = hampel(frames["/sim/speed"]["accel"], window_size=10)
        outliers = frames["/sim/speed"].iloc[hampel_accel.outlier_indices]
        filtered_accel = frames["/sim/speed"].drop(outliers.index)
        savgol_accel = savgol_filter(filtered_accel["accel"], 50, 3)
        axs[iteration-1,2].plot(filtered_accel["accel"], color="orange")
        axs[iteration-1,2].plot(filtered_accel.index, savgol_accel, color="darkorange")

        axs[iteration-1,3].set_xlim([-1,1])
        axs[iteration-1,3].set_ylim([-1,1])
        axs[iteration-1,3].text(0,0,"t_"+str(throttle)+"_"+str(iteration), horizontalalignment="center",verticalalignment="center")
        """

    plt.show()
    plt.gcf().set_size_inches(15, 12)
    plt.savefig(path_prefix+str(throttle)+".png")
    print(f"saved plot for throttle {throttle}")