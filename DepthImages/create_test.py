# import the necessary modules
import freenect
import numpy as np
import sys
import time
import math
from scipy.misc import imshow
from PIL.Image import fromarray


# NOTE THIS IS CURRENTLY BUGGED DUE TO THE FIRST LINE BEING
# PRINTED INCORRECTLY

def get_video():
    array, _ = freenect.sync_get_video()
    return array

def get_depth():
    array, _ = freenect.sync_get_depth()
    return array


if __name__ == "__main__":
    f = open('middlepixel.txt', 'w')
    while 1:
        keep_images = "n"
        while keep_images != "y":
            # Get a frame from RGB camera & depth sensor
            array = get_depth()
            depth = array.astype(np.uint8)
            color = get_video()

            # Display captured frames
            # imshow(array)
            # imshow(color)
            
            keep_images = raw_input("Do you wish to keep these images? (y/n): ")

        

        # Get the test number we're going to create
        with open("answers.txt") as ans:
            answers = ans.readlines()
        ans.close()

        answers[0] = str(int(answers[0]) + 1)
        test_count = int(answers[0])

        # Create output files
        f = open("test" + str(test_count) + '.txt', 'w')
        h, w = np.shape(array)
        line = ""

        for py in range(0, h):  # height
            for px in range(0, w):  # width
                maths = 0.1236 * math.tan((array[py][px] / 2842.5) + 1.1863)
                line += str(maths) + " "
            f.write(line + "\n\n")
            line = ""
        f.close()

        color_img = fromarray(color)
        color_img.save('test{}color.png'.format(test_count))
		
        depth_img = fromarray(depth)
        depth_img.save('test{}depth.png'.format(test_count))

        is_obstacle = raw_input("Is there an obstacle present? (True/False): ")
        distance = raw_input("Please input the distance of the object in mm (any if there is none): ")

        answers.append("\n\n{} {}".format(is_obstacle, str(distance)))

        with open("answers.txt", 'w') as ans:
            ans.writelines(answers)
        ans.close()
