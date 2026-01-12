import struct
import mmap

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

# Example of reading a Vote from a memory-mapped file
with open('_data0', 'r+b') as f:
    mm = mmap.mmap(f.fileno(), 0)

    while True:
        mm.seek(0)
        # Read a Vote from the memory-mapped file
        vote_data = mm.read(vote_size)
        print(len(vote_data))
        vote = struct.unpack(vote_format, vote_data)

        # Extract the idx and MappedJointTrajectoryPoint value
        idx = vote[0]
        mapped_joint_trajectory_point = vote[1:]

        # Extract positions and velocities arrays
        positions_length = mapped_joint_trajectory_point[0]
        positions = mapped_joint_trajectory_point[1:101]  # 100 doubles for positions
        velocities_length = mapped_joint_trajectory_point[101]
        velocities = mapped_joint_trajectory_point[102:202]  # 100 doubles for velocities

        print(f"Index: {idx}")
        print(f"Positions Length: {positions_length}")
        print(f"Positions: {positions}")
        print(f"Velocities Length: {velocities_length}")
        print(f"Velocities: {velocities}")

    # Close the memory-mapped file
    mm.close()