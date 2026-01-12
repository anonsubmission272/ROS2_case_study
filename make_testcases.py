import csv
import sys
import random
import os

# LINE_LEN = 7
REPAIR_PATH = "."
MIN_IDX = 1
NUM_PASS = 2
NUM_FAIL = 2

NUM_TESTS = 2

# pass in the controller number
bad = sys.argv[1]

max_idx = int(sys.argv[2])

# with open("missed_" + bad + ".txt") as missed:
#     miss_lines = missed.readlines()
#     miss = []
#     for x in miss_lines:
#         idx = int(x[:-1])
#         if idx >= MIN_IDX:
#             miss.append(idx)

#     # miss = [x[:-1] for x in miss_lines]
#     print("controller missed: " + str(miss))


# if max_idx - MIN_IDX < NUM_TESTS:
#     print("ERROR: not enough recorded data")
#     # exit(1)

# potential_passes = range(MIN_IDX, max_idx)

# passes = [x for x in potential_passes if x not in miss]

# print(f"passes: {passes}")


# miss_cases = random.sample(miss, NUM_FAIL)

# print(miss_cases)



# get the last 10 recorded cases including max_idx
for i in range(NUM_TESTS):
    os.system(f"cp results/state_{max_idx - NUM_TESTS + i + 1} {REPAIR_PATH}/docker/test/n1/t{i + 1}")
    os.system(f"cp results/actuation_{max_idx - NUM_TESTS + i + 1} {REPAIR_PATH}/docker/test/n1/output.t{i + 1}")
    # print(f"max idx: {max_idx}     i: {i}")


# for i in range(len(miss_cases)):
#     os.system(f"cp results/state_{miss_cases[i]} {REPAIR_PATH}/docker/test/n{i + 1}")
#     os.system(f"cp results/actuation_{miss_cases[i]} {REPAIR_PATH}/docker/test/output.n{i + 1}")

# pass_cases = random.sample(passes, NUM_PASS)

# print(pass_cases)

# for i in range(len(pass_cases)):
#     os.system(f"cp results/state_{pass_cases[i]} {REPAIR_PATH}/docker/test/p{i + 1}")
#     os.system(f"cp results/actuation_{pass_cases[i]} {REPAIR_PATH}/docker/test/output.p{i + 1}")

