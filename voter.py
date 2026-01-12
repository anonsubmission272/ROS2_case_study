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
import time

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

myIdx = -1

# Define A and trust_scores globally
A = [0,0,0]  
trust_scores = [0.75] * len(A)
# trust_scores[0] = 0.4
active_controllers =  [True] * len(A)
#controller_paths = [f'c{i}/controller.c' for i in range(len(A))]

trajectory_points = [None] * len(A)
leader = None

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



class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)
            
def modify_voter_positions(A):
    # Modify each position in A by adding a random value between -1 and 1
    return [a + random.uniform(-1, 1) for a in A]

def update_trust_scores(A, accepted_votes):
    # Placeholder for logic to update trust scores
    # Update 'scores' based on 'some_logic'

    global trust_scores, active_controllers
    for (idx,vote) in enumerate(A):
        # if active_controllers[idx]:
            # deviation = abs(vote - accepted_value)
        if (idx,vote) in accepted_votes:
            trust_scores[idx] = min(trust_scores[idx] + 0.03, 1.0) #= min(trust_scores[idx] + 0.5 * ((1 - trust_scores[idx]) / (1 + deviation)), 1)
            print(f"controller {idx} voted correctly")
        else:
            # take off at most 0.1, scaled by how wrong it is
            trust_scores[idx] = max(trust_scores[idx] - 0.08, 0.0) #= max(trust_scores[idx] - deviation / 100, 0)
            print(f"crontroller {idx} voted wrong")
            write_missed(idx)


def write_missed(idx):
    with open(f'missed_{idx}.txt', 'a') as file:
        file.write(str(myIdx) + "\n")

def delete_missed_files(directory):

    # Construct the path pattern to match all 'missed_*.txt' files in the specified directory

    path_pattern = os.path.join(directory, 'missed_*.txt')

    # Find all files matching the pattern

    files = glob.glob(path_pattern)

    # Remove each file found

    for file in files:

        os.remove(file)

        print(f"Deleted file: {file}")


def check_trust(current_trust_scores, threshold):
    global active_controllers
    for i, score in enumerate(current_trust_scores):
        if active_controllers[i] == True and score < threshold:
            active_controllers[i] = False
            print(f"sending controller {i} for fixing")
            # DEBUG
            exit(i)
            # make a thread to call for repair here on that controller path_to_controller_i
            thread = threading.Thread(target=repair_controller, args=(i,))
            thread.start()
            # assume there is a table/array with paths to controllers
            # thread dies, pass in i

def repair_controller(controller_index):
    #controller_path = controller_paths[controller_index]  # Assuming controller_paths is defined elsewhere
    # Call the external repair script with the path to the controller as an argument
    result = subprocess.run(['bash', 'repair.sh', str(controller_index), str(myIdx)], capture_output=True, text=True)
    if result.returncode == 0:
        # If the script completes successfully, reactivate the controller
        #active_controllers[controller_index] = True
        trust_scores[controller_index] = 0.75 # reset trust score
        print(f"Controller {controller_index} repaired and reactivated.")

        #subprocess.run("cp patches/0.diff patches_"+str(controller_index)+"/0.diff",shell=True, check=True)
        os._exit(0)

    else:
        # Handle cases where the script fails
        print(f"Failed to repair controller {controller_index}. Error: {result.stderr}")
        os._exit(1)

# FMV
def vote(A, epsilon):
    global active_controllers, leader
    # Initializes a list to hold subdivisions, each subdivision is a list of outputs
    subdivisions = []

    # Iterate over each output in A
    for idx, x in enumerate(A):
        if active_controllers[idx]:
            if x is None:
                # FIXME this is just for testing and debugging
                # DEBUG should actually handle missed votes better than this
                print(f"controller {idx} missed the vote")
                # if(myIdx > 10):
                #     trust_scores[idx] -= 0.1
                #     write_missed(idx)
                continue
            # A flag to check if x has been added to a subdivision
            added_to_subdivision = False
            
            # Check each existing subdivision to see if x fits into it
            for subdivision in subdivisions:
                # If x is within epsilon of any element in the subdivision, add x to this subdivision

                # DEBUG
                for y in subdivision:
                    print(f"cosine distance of x and y: {cosine_distance(x, y[1])}")

                if any(cosine_distance(x, y[1]) <= epsilon for y in subdivision):
                    subdivision.append((idx,x))
                    added_to_subdivision = True
                    break
            
            # If x does not fit into any existing subdivision, create a new one for x
            if not added_to_subdivision:
                
                subdivisions.append([(idx,x)])

    # print('subd', subdivisions)
    # Find the largest subdivision
    if len(subdivisions) > 0:
        largest_subdivision = max(subdivisions, key=len)

        accepted_idx = None
        subdiv_idx_list = [x[0] for x in largest_subdivision]

        if leader in subdiv_idx_list:
            accepted_idx = leader
        else:
            accepted_idx = random.choice(subdiv_idx_list)
            leader = accepted_idx
        # FIXME no averages!
        # Calculate and return the average of the largest subdivision
        # average_value = sum(y[1] for y in largest_subdivision) / len(largest_subdivision)
        
        accepted_votes = set(largest_subdivision)
        update_trust_scores(A, accepted_votes)
        
        # print('avg val', average_value)
        return accepted_idx
    else:
        print("There are no subdivisions")
        return 0

def driver(data0, data1, data2, actuation):
    global myIdx 
    global A, trust_scores  # Indicate that we're using the global variables
    #A = modify_voter_positions(A)  # Update A with modified positions
    print(f"My Index Is: {myIdx}")
    try:
        data0.seek(0)
        myVote  = struct.unpack(vote_format,data0.read(vote_size))
         # Extract the idx and MappedJointTrajectoryPoint value
        vidx = myVote[0]

        print(f"Controller 0 index: {vidx}")

        if(vidx > myIdx):
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
            trajectory_points[0] = mapped_joint_trajectory_point

            A[0] = positions[0:6] + velocities[0:6]
        else:
            A[0] = None
    except struct.error:
        print("could not read file 0")

    try:
        data1.seek(0)
        myVote  = struct.unpack(vote_format,data1.read(vote_size))
         # Extract the idx and MappedJointTrajectoryPoint value
        vidx = myVote[0]

        print(f"Controller 1 index: {vidx}")

        if(vidx > myIdx):
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
            trajectory_points[1] = mapped_joint_trajectory_point

            A[1] = positions[0:6] + velocities[0:6]
        else:
            A[1] = None
    except struct.error:
        print("could not read file 1")

    try:
        data2.seek(0)
        myVote  = struct.unpack(vote_format,data2.read(vote_size))
         # Extract the idx and MappedJointTrajectoryPoint value
        vidx = myVote[0]

        print(f"Controller 2 index: {vidx}")

        if(vidx > myIdx):
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
            trajectory_points[2] = mapped_joint_trajectory_point

            A[2] = positions[0:6] + velocities[0:6]
        else:
            A[2] = None
    except struct.error:
        print("could not read file 2")

    epsilon = 0.5 # slightly larger than noise
    
    output = vote(A, epsilon)

    with open("results.csv", 'a') as f:
        f.write(str(output) + "\n")
        # f.flush()
        # os.fsync(f.fileno())
    
    print(A)  # Print updated A for verification
    print(trust_scores)  # Print updated trust scores for verification
    
    threshold = 0.5
    check_trust(trust_scores, threshold)
    # myIdx = 69
    # output = 69.69 
    
    myIdx += 1

    vote_data_to_write = struct.pack(vote_format, myIdx, *trajectory_points[output])

    actuation.seek(0)
    actuation.write(vote_data_to_write)

    # copy the state and actuation to the results
    os.system(f"./copy_results.sh {myIdx}")

    
    
if __name__ == "__main__":
    # Directory containing the files
    directory = "./"  # Update this to your specific folder path if different
    # Delete all 'missed_*.txt' files before running the main function
    delete_missed_files(directory)

    #print(trust_scores)
    with open("_data0", "rb") as d0, open("_data1", "rb") as d1, open("_data2", "rb") as d2, open("_actuation", "w+b") as a:
            #a.write(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            a.seek(0)
            a.write(b'\x00' * vote_size)
            a.flush()
            # a.write(bytes(16))
            a.seek(0)
            d0.seek(0)
            d1.seek(0)
            d2.seek(0)

            # Create memory maps for data0 and actuation
            data0 = mmap.mmap(d0.fileno(), 0, access=mmap.ACCESS_READ)
            data1 = mmap.mmap(d1.fileno(), 0, access=mmap.ACCESS_READ)
            data2 = mmap.mmap(d2.fileno(), 0, access=mmap.ACCESS_READ)
            actuation = mmap.mmap(a.fileno(), 0, access=mmap.ACCESS_WRITE)

            # print(data0)
            # print(actuation)
            
            flag_path = "_flag"

            with open("_time", "w") as t:
                while True:
                    if os.path.exists(flag_path):
                        print("New vote cycle beginning...")

                        start_time = time.perf_counter_ns()
                        driver(data0, data1, data2, actuation)
                        os.remove(flag_path)
                        end_time = time.perf_counter_ns()
                        tot_time = end_time - start_time

                        t.write(str(tot_time) + "\n")


            # # Start timer
            # t = RepeatTimer(0.05, driver, [data0, data1, data2, actuation])
            # t.start()

    # index of what vote we are on, track which vote indexes it got wrong - to know which ones are test cases, write to a file
    # integrate trust scores
    # detect when a controller breaks/which one
    # add run bash script 
    # integrate into a control loop system
    # input and output 
    # integrate internal state vote???????????
    # of cycles before detection given certain hyperparameters
    # hyperparameters - conservative, moderate, and how long it takes to detect
    # x axis how wrong the controller is y axis how long it took
    # wrong every time, 1/x times and by how much
    