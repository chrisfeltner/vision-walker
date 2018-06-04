from scipy import stats
import numpy as np

from settings import I_THRESHOLD, A_THRESHOLD, B_THRESHOLD

class Detector(object):
    def get_average_values(self, width_for_avg, array):
        assert width_for_avg != 0
        height, width = array.shape
        center = width/2
        starting_width_index = center - (width_for_avg/2)
        ending_width_index = center + (width_for_avg/2) + width_for_avg%2
        average_array = np.zeros(height)
        number_of_elements = ending_width_index - starting_width_index
        if ending_width_index == starting_width_index:
            number_of_elements = 1
        for y in range(0, height):
            sum = 0.0
            for x in range(starting_width_index, ending_width_index):
                sum += array[y][x]
            average_array[y] = sum/number_of_elements
        return average_array


    # Detects obstables by identifying non-linear distance
    # Returns True if an obstacle is detected
    def detect_obstacle(self, averaged_array):
        indicies = np.arange(1, len(averaged_array) + 1)
        slope, intercept, r_value, p_value, std_err = stats.mstats.linregress(indicies, array)

        if (r_value * r_value) > I_THRESHOLD and slope < A_THRESHOLD and  intercept > B_THRESHOLD:
            return False
        else:
            return True

    def get_distance_from_object(self, averaged_array):
        pass

    def linear_segmentation(self, averaged_array):
        return self.sliding_window_segmentation(averaged_array, np.std(averaged_array)/2)

    def sliding_window_segmentation(self, averaged_array, max_error):
        breakpoints = [0]
        anchor = 0
        i = 1
        while anchor < averaged_array.shape[0] - 1:
            i = 1
            while abs(np.std(averaged_array[anchor:anchor + i])) < max_error and anchor + i < averaged_array.shape[0]:
                i = i + 1
            breakpoints.append(anchor + i - 1)
            anchor = anchor + i - 1
        return breakpoints

    def bottom_up_segmentation(self, averaged_array, max_error):
        pass


if __name__ == '__main__':
    dt = Detector()
    array = np.array([1,2,3,4,5, 250, 251, 252, 6, 7, 8])
    print(np.std(array))
    print(dt.linear_segmentation(array))
