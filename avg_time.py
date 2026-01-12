import sys

time_file = sys.argv[1]

with open(time_file) as f:
    time_strings = f.readlines()

times = [int(x) for x in time_strings]
avg_time = sum(times)/len(times)

print(avg_time)