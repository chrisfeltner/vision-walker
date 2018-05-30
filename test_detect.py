from scipy import stats
import numpy as np
import sys


def detect(input_file, i_threshold, a_threshold, b_threshold):
    # file_name = sys.argv[1]
    # print(file_name)
    array = np.loadtxt(input_file)
    array = array[array != 0]
# Need to rework ^ to work with 2d arrays?
    indicies = np.arange(1, len(array) + 1)

    slope, intercept, r_value, p_value, std_err = stats.mstats.linregress(indicies, array)

    # print("Slope: " + str(slope))
    # print("Intercept: " + str(intercept))
    # print("R: " + str(r_value))
    # print("R Squared: " + str(r_value * r_value))
    # print("P: " + str(p_value))
    # print("Std Error: " + str(std_err))

    # i_threshold = 0.4
    # a_threshold = -4
    # b_threshold = 3000

    if (r_value * r_value) > i_threshold and slope < a_threshold and intercept > b_threshold:
        return False  # NO OBJECT DETECTED
    else:
        return True  # OBJECT DETECTED
