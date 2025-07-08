import os
import rowan
import numpy as np
import robotic as ry

qs = []
pcs = []

for file_name in sorted(os.listdir("log_data")):
    file_path = os.path.join("log_data", file_name)
    with open(file_path, 'r') as f:
        lines = f.readlines()

        # Parse the first line
        first = list(map(float, lines[0].strip().split()))
        if len(first) != 3:
            print(f"Warning: First line in {file_name} is not 3 floats. Skipping file.")
            continue
        qs.append(first)

        # Parse remaining lines
        rest = []
        for line in lines[1:]:
            # if np.random.random() < .01:
            if np.random.random() < 1:
                nums = list(map(float, line.strip().split()))
                if len(nums) == 3:
                    rest.append(nums)
                else:
                    print(f"Warning: Skipping malformed line in {file_name}: {line.strip()}")
        pcs.append(rest)

timestamp_count = len(qs)
point_cloud_size = len(pcs[0])
print("Total Timestamps: ", timestamp_count)
print("Point Cloud Size: ", point_cloud_size)

C = ry.Config()

for i, q in enumerate(qs):

    pos = [q[0], q[1], .1]
    quat = rowan.from_euler(0, 0, q[2], convention="xyz")

    C.addFrame(f"pose{i}") \
        .setShape(ry.ST.marker, [.3]) \
        .setQuaternion(quat) \
        .setPosition(pos)

    C.addFrame(f"point_cloud{i}", f"pose{i}") \
        .setPointCloud(pcs[i]) \
        .setColor([0., 0., 1., .05])

for i in range(timestamp_count):
    C.getFrame(f"point_cloud{i}").setColor([1., 0., 0., 1.])
    C.view(True)
    C.getFrame(f"point_cloud{i}").setColor([0., 0., 1., .05])
