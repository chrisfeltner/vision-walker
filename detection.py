from scipy import stats
import numpy as np

from settings import I_THRESHOLD, A_THRESHOLD, B_THRESHOLD

class Detector(object):
    def get_average_values(width_for_avg, array):
        height, width = np.shape(array)
        center = width/2
        starting_width_index = center - (width_for_avg/2)
        ending_width_index = center + (width_for_avg/2)
        average_array = np.zeros(height)
        for y in range(0, height):
            sum = 0
            for x in range(starting_width_index, ending_width_index):
                sum += array[y][x]
            average_array[y] = sum/(ending_width_index - starting_width_index)
        return average_array


    # Detects obstables by identifying non-linear distance
    # Returns True if an obstacle is detected
    def detect_obstacle(averaged_array):
        indicies = np.arange(1, len(averaged_array) + 1)
        slope, intercept, r_value, p_value, std_err = stats.mstats.linregress(indicies, array)

        if (r_value * r_value) > I_THRESHOLD and slope < A_THRESHOLD and  intercept > B_THRESHOLD:
            return False
        else:
            return True

    def get_distance_from_object(averaged_array):

