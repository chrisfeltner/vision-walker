from scipy import stats
import numpy as np
import sys


def detect(input_file, i_threshold, a_threshold, b_threshold):
    array = np.loadtxt(input_file)
    # Temporary fix for the 2d array stuff. In practice we wouldn't need this?
    if len(np.shape(array)) > 1:
        height, width = np.shape(array)
        array = array[:, width / 2]
        print("Height {} Width {} size {}".format(height, width, len(array)))
    array = array[array != 0]
    if len(array) == 0:
        # For now we can assume super close object filling screen = bad
        return True

    indicies = np.arange(1, len(array) + 1)
    slope, intercept, r_value, p_value, std_err = stats.mstats.linregress(
        indicies, array)

    if (r_value * r_value) > i_threshold and slope < a_threshold and intercept < b_threshold:
        return False  # NO OBJECT DETECTED
    else:
        return True  # OBJECT DETECTED
