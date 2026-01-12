import random
from threading import Timer
import struct
import mmap
import threading
import subprocess
import os
import glob
import numpy as np
from numpy.linalg import norm
import sys

#import time

# Define the format string for MappedJointTrajectoryPoint
mapped_joint_trajectory_point_format = (
    'Q' +         # positions_length (size_t)
    '100d' +      # positions (100 doubles)
    'Q' +         # velocities_length (size_t)
    '100d' +      # velocities (100 doubles)
    'Q' +         # accelerations_length (size_t)
    '100d' +      # accelerations (100 doubles)
    'Q' +         # effort_length (size_t)
    '100d' +      # effort (100 doubles)
    'i' +         # time_from_start_sec (int32_t)
    'I'           # time_from_start_nsec (uint32_t)
)

# Calculate the size of MappedJointTrajectoryPoint
mapped_joint_trajectory_point_size = struct.calcsize(mapped_joint_trajectory_point_format)

# Define the format string for Vote
vote_format = (
    'i' +         # idx (int)
    mapped_joint_trajectory_point_format  # value (MappedJointTrajectoryPoint)
)

# Calculate the size of Vote
vote_size = struct.calcsize(vote_format)

A = [None,None]

def cosine_distance(vec1, vec2):
    """
    Calculates the cosine similarity between two vectors.

    Args:
    vec1 (list or numpy.ndarray): The first vector.
    vec2 (list or numpy.ndarray): The second vector.

    Returns:
    float: The cosine similarity between the two vectors.
    """
    vec1 = np.array(vec1)
    vec2 = np.array(vec2)
    norm1 = norm(vec1)
    norm2 = norm(vec2)
    # FIXME - tmp fix
    if norm1 == 0 and norm2 == 0:
        # one of the vectors is 0 so they are not similar... idk if this is right.. ask kevin
        return 0
    elif norm1 == 0 or norm2 == 0:
        return 1
    return 1 - (np.dot(vec1, vec2) / (norm(vec1) * norm(vec2)))


# FMV
def check_dist(A, epsilon):
    if cosine_distance(A[0], A[1]) <= epsilon:
        return True
    else:
        return False
    

def driver(data, oracle):
    global myIdx 
    global A, trust_scores  # Indicate that we're using the global variables
    #A = modify_voter_positions(A)  # Update A with modified positions
    # print(f"My Index Is: {myIdx}")
    try:
        data.seek(0)
        myVote  = struct.unpack(vote_format,data.read(vote_size))
         # Extract the idx and MappedJointTrajectoryPoint value
        vidx = myVote[0]

        # print(f"Controller 0 index: {vidx}")


        mapped_joint_trajectory_point = myVote[1:]

        # Extract positions and velocities arrays
        positions_length = mapped_joint_trajectory_point[0]
        positions = mapped_joint_trajectory_point[1:101]  # 100 doubles for positions
        velocities_length = mapped_joint_trajectory_point[101]
        velocities = mapped_joint_trajectory_point[102:202]  # 100 doubles for velocities

        # print(f"Index: {vidx}")
        # print(f"Positions Length: {positions_length}")
        # print(f"Positions: {positions}")
        # print(f"Velocities Length: {velocities_length}")
        # print(f"Velocities: {velocities}")

        # Save the trajectory point for later

        A[0] = positions[0:6] + velocities[0:6]

    except struct.error:
        print("could not read file 0")

    try:
        oracle.seek(0)
        myVote  = struct.unpack(vote_format,oracle.read(vote_size))
         # Extract the idx and MappedJointTrajectoryPoint value
        vidx = myVote[0]

        print(f"Controller 1 index: {vidx}")

        mapped_joint_trajectory_point = myVote[1:]

        # Extract positions and velocities arrays
        positions_length = mapped_joint_trajectory_point[0]
        positions = mapped_joint_trajectory_point[1:101]  # 100 doubles for positions
        velocities_length = mapped_joint_trajectory_point[101]
        velocities = mapped_joint_trajectory_point[102:202]  # 100 doubles for velocities

        # print(f"Index: {vidx}")
        # print(f"Positions Length: {positions_length}")
        # print(f"Positions: {positions}")
        # print(f"Velocities Length: {velocities_length}")
        # print(f"Velocities: {velocities}")
        # Save the trajectory point for later

        A[1] = positions[0:6] + velocities[0:6]

    except struct.error:
        print("could not read file 1")


    epsilon = 0.5 # slightly larger than noise
    
    return check_dist(A, epsilon)

    
    


if __name__ == "__main__":
    oracle_path = sys.argv[1]

    #print(trust_scores)
    with open("data", "rb") as d, open(oracle_path, "rb") as o:
        #a.write(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        d.seek(0)
        o.seek(0)

        # Create memory maps for data0 and actuation
        data = mmap.mmap(d.fileno(), 0, access=mmap.ACCESS_READ)
        oracle = mmap.mmap(o.fileno(), 0, access=mmap.ACCESS_READ)

        if driver(data,oracle):
            exit(0)
        else:
            exit(1)
