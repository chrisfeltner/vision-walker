from settings import I_THRESHOLD, A_THRESHOLD, B_THRESHOLD

class Detector(object):
    def get_average_values(width, array):

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