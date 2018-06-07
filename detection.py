from scipy import stats
import numpy as np

import matplotlib.pyplot as plot

from settings import I_THRESHOLD, A_THRESHOLD, B_THRESHOLD, ERROR_COEFFICIENT

class Detector(object):

    # Given a 2D array and a width, this function will return a numpy array with
    # the average of each row from width_for_avg/2 before the center column and 
    # width_for_avg/2 after the cetner column. For example, if the 2D array is
    # an image, this function will return the average pixel values of the pixels
    # width_for_avg/2 pixels or less away from the center line of the image. It does
    # NOT consider values of 0 in the average since this is used as an error state by
    # the Kinect camera.
    def get_average_values(self, width_for_avg, array):
        assert width_for_avg != 0
        height, width = array.shape
        center = width/2
        starting_width_index = center - (width_for_avg/2)
        ending_width_index = center + (width_for_avg/2) + width_for_avg%2
        average_array = np.zeros(height)
        for y in range(0, height):
            sum = 0.0
            number_of_elements = 1
            for x in range(starting_width_index, ending_width_index):
                if array[y][x] != 0:
                    sum += array[y][x]
                    number_of_elements = number_of_elements + 1
            average_array[y] = sum/number_of_elements
        return average_array


    # Detects obstables by identifying non-linear distance
    # Returns True if an obstacle is detected
    def detect_obstacle(self, averaged_array):
        indicies = np.arange(1, len(averaged_array) + 1)
        slope, intercept, r_value, p_value, std_err = stats.mstats.linregress(indicies, array)

        if (r_value * r_value) > I_THRESHOLD and slope < A_THRESHOLD and  intercept < B_THRESHOLD:
            return False
        else:
            return True

    # This will return the minimum distance from an object recognized by
    # linear piecewise segmentation
    def get_distance_from_object(self, averaged_array):
        segments = self.linear_segmentation(averaged_array)
        if len(segments) > 2:
            distances = []
            for i in range(0, len(segments) - 1):
                distances.append(averaged_array[segments[i]])
            return min(distances)
        else:
            return -1

    # This function will run Sliding Window Segmentation on the given array using std error
    # multiplied by a coefficient for the measure of max_error
    # You should run this function on an array after you have called get_average_values
    # on that array.
    def linear_segmentation(self, averaged_array):
        return self.sliding_window_segmentation(averaged_array, np.std(averaged_array) * ERROR_COEFFICIENT)

    # Given a 1D numpy array, this function will remove values of 0.0 from the array
    # It will return the resulting array
    def remove_zero_values(self, averaged_array):
        zero_indices = []
        for index in range(0, len(averaged_array)):
            if(averaged_array[index] == 0):
                zero_indices.append(index)
        return np.delete(averaged_array, zero_indices)

    # Given a 1D numpy array and a max error value, this function will return a list of
    # indicies where the Sliding Window Segmentation function recognizes a segment
    def sliding_window_segmentation(self, averaged_array, max_error):
        breakpoints = [0]
        anchor = 0
        while anchor < averaged_array.shape[0] - 2:
            i = 1
            j = 0
            while abs(np.std(averaged_array[anchor + j:anchor + i])) < max_error and anchor + i < averaged_array.shape[0]:
                i = i + 1
                if anchor + i > 150:
                    j = j + 1
            breakpoints.append(anchor + i - 1)
            anchor = anchor + i - 1
        return breakpoints

    def plot_segmentation(self, averaged_array):
        self.remove_zero_values(averaged_array)
        segments = self.linear_segmentation(averaged_array)
        plot.plot(averaged_array)
        for segment in segments:
            plot.axvline(x=segment, color='k', linestyle='--')
        plot.show()       

    def bottom_up_segmentation(self, averaged_array, max_error):
        # segments = []
        # merge_cost = []
        # for i in xrange(0, len(averaged_array), 2):
        #     segments.append(i)
        # for segment in range(0, len(segments) - 2):
        #     merge_cost.append(np.std(averaged_array[segment, segment + 2]))

        # while min(merge_cost) < max_error:
        pass



if __name__ == '__main__':
    dt = Detector()
    array = dt.get_average_values(25,np.loadtxt('data/2.txt'))
    print(np.std(array))
    print(dt.remove_zero_values(array))
    array = dt.remove_zero_values(array)
    segments = dt.linear_segmentation(array)
    print(segments)
    print(dt.get_distance_from_object(array))
    dt.plot_segmentation(array)
