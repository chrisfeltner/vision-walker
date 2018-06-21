import numpy as np


def detect(input, width, threshold):
    array = input
    if len(np.shape(array)) > 1:
        h, w = np.shape(array)
        array = array[:, w / 2]
        #print("Height {} Width {} size {}".format(h, w, len(array)))
    array = array[array != 0]
    if len(array) == 0 or len(array) < 260:
        # For now we can assume super close object filling screen = bad
        #print("Distance is 1")
        #print("Len is " + str(len(array)))
        return 1

    array = array[::-1]

    index = 0

    distance = 9999

    while index < len(array) - width:
        x1 = index
        y1 = array[index]
        x2 = index + width
        y2 = array[index + width]

        #print("{} {} {} {}".format(x1, x2, y1, y2))

        slope = (y2 - y1) / (x2 - x1)
        #print(slope)

        index += width
        if slope < 0:
            break

    while index < len(array):
        if array[index] < distance:
            distance = array[index]
        index += 1
        
    #print("Distance is " + str(distance))
    if distance > threshold:
        return -1  # NO OBJECT DETECTED
    else:
        return distance  # OBJECT DETECTED


def detect_file(input_file, width, threshold):
    array = np.loadtxt(input_file)
    if len(np.shape(array)) > 1:
        h, w = np.shape(array)
        array = array[:, w / 2]
        # print("Height {} Width {} size {}".format(height, width, len(array)))
    array = array[array != 0]
    if len(array) == 0 or len(array) < 300:
        # For now we can assume super close object filling screen = bad
        #print("Distance is 1")
        return 1

    array = array[::-1]

    index = 0

    distance = 9999

    while index < len(array) - width:
        x1 = index
        y1 = array[index]
        x2 = index + width
        y2 = array[index + width]

        #print("{} {} {} {}".format(x1, x2, y1, y2))

        slope = (y2 - y1) / (x2 - x1)
        #print(slope)

        index += width
        if slope < 0:
            break

    while index < len(array):
        if array[index] < distance:
            distance = array[index]
        index += 1
        
    #print("Distance is " + str(distance))
    if distance > threshold:
        return -1  # NO OBJECT DETECTED
    else:
        return distance  # OBJECT DETECTED
        
