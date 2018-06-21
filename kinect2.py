import freenect
import detect2
import numpy as np


def get_depth():
    array, _ = freenect.sync_get_depth(format=freenect.DEPTH_MM)
    return array

if __name__ == "__main__":
    while 1:
        array = get_depth()
        detection_result = detect2.detect(array, 2, 2000)

        if detection_result == -1:
            print("No object detected!")
        elif detection_result > 1:
            print("Object detected, {}mm away!".format(detection_result))