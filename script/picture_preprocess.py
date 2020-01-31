import sys
import os
import numpy as np


def preprocess(pps_event_path, picture_path):
    pps_info = np.loadtxt(pps_event_path)
    pps_info = pps_info[:][2:4]


if __name__ == "__main__":
    if (len(sys.argv) != 3):
        print(
            "usage: python picture_preprocess pps_event.txt picture_name"
        )
        exit()
    preprocess(sys.argv[1], sys.argv[2])
