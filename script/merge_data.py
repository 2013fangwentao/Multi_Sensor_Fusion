import numpy as np
import os as os
import sys


def merge_kitti_gpsimu(data_path, timestamps_file, output_path):
    timestamps = np.loadtxt(timestamps_file)
    row_size = timestamps.shape[0]
    time_size = timestamps.shape[1]
    processed_data = np.zeros([row_size, time_size + 14])
    for dir_path, subpaths, files in os.walk(data_path, False):
        if (row_size != len(files)):
            print("timestamps error")
            exit()
        count = 0
        for file in files:
            per_data = np.loadtxt(os.path.join(data_path, file))
            processed_data[count][0:6] = timestamps[count][:]
            processed_data[count, time_size:time_size + 3] = per_data[0:3]
            processed_data[count, time_size + 3:time_size +
                           6] = per_data[17:20]
            processed_data[count, time_size + 6:time_size +
                           9] = per_data[11:14]
            processed_data[count, time_size + 9:time_size + 12] = per_data[3:6]
            processed_data[count, time_size + 12:time_size +
                           14] = per_data[6:8]
            count += 1
    np.savetxt(
        os.path.join(output_path, "./merge_data.txt"), processed_data, "%.10f",
        "\t")


if __name__ == "__main__":
    if (len(sys.argv) != 4):
        print(
            "usage: python merge_data gpsimu_data_path timestamps_file output_path"
        )
        exit()
    merge_kitti_gpsimu(sys.argv[1], sys.argv[2], sys.argv[3])
