import numpy as np
import config


# image - Depth image file. (For testing)
# width - Column width to detect
def change_this_name(image, width):
    # array = np.loadtxt(image_file)

    h, w = np.shape(image)
    middle = (int)(w / 2)

    closest_distance = -1

    for column in range(middle - width, middle + width):
        column_distance = -1
        column_array = image[:, column]

        column_distance = detect(2, image=column_array)

        if column == middle:
            print("Normally {} would be your output".format(column_distance))

        if column_distance != -1:
            if closest_distance == -1:
                # print("Before: Column distance is {} closest distance is {}".format(column_distance, closest_distance))
                closest_distance = column_distance
            elif closest_distance > column_distance and column_distance > 100:
                closest_distance = column_distance
        # print("After: Column distance is {} closest distance is {}".format(column_distance, closest_distance))
        # print("Closest distance is {}".format(closest_distance))
    # print("Going to return {}".format(closest_distance))
    if closest_distance > config.MAX_DISTANCE:
        return -1

    return closest_distance


# input - Depth image.
# width - Determines where we pick the y2/x2 for the slope calculation.
def detect(width, image=None, image_file=None):
    if image_file is None:
        array = image
    else:
        array = np.loadtxt(image_file)

    # Only required if we're not averageing the array by this point.
    # Gets the 640 x 480 array and turns it into a 1-d array across the middle.
    if len(np.shape(array)) > 1:
        h, w = np.shape(array)
        array = array[:, w / 2]

    # Remove 0s, as they are error values.
    array = array[array != 0]

    # For now we assume super close object mostly filling screen = bad.
    if len(array) < 200:
        return 1

    # Flip the array so that we have it in a more intuitive order.
    # Camera is flipped on walker, not currently needed.
    # array = array[::-1]

    # Index, used to keep track of where we break out of the while loop.
    index = 0

    # Current closest distance, updated if we break out of the 1st while loop.
    distance = 9999

    while index < len(array) - width:
        # Slope calculation
        x1 = index
        y1 = array[index]
        x2 = index + width
        y2 = array[index + width]

        slope = (y2 - y1) / (x2 - x1)

        index += width

        # Break out of the loop if we have a non-positive slope.
        if slope < -2:
            break

        if slope > 200:
            return y1

    # Continue where we left off from ^
    # Find the lowest y-value, corresponding to the closest point.
    while index < len(array):
        if array[index] < distance:
            distance = array[index]
        index += 1
    if config.DEBUG:
        print("Debug distance is {}".format(distance))
    # We've found an object if the distance is less than our threshold.
    if distance > config.MAX_DISTANCE:
        return -1  # NO OBJECT DETECTED
    else:
        return distance  # OBJECT DETECTED