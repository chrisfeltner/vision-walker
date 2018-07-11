# import the necessary modules
import freenect
import numpy as np
import sys
import time
import math
from scipy.misc import imshow
from PIL.Image import fromarray

# function to get RGB image from kinect


def get_video():
    array, _ = freenect.sync_get_video()
    return array

# function to get depth image from kinect


def get_depth():
    array, _ = freenect.sync_get_depth()
    return array


if __name__ == "__main__":
    f = open('middlepixel.txt', 'w')
    while 1:
        # get a frame from RGB camera
        # frame = get_video()
        # get a frame from depth sensor
        array = get_depth()
        depth = array.astype(np.uint8)
        color = get_video()

        # depth = depth.astype(np.uint16)
        # display cv2.imshow('RGB image',cv2.Canny(frame, 150, 200))
        # display depth image
        # cv2.imshow('Depth image',depth)
        imshow(array)
        imshow(color)
        f = open('imageoutput.txt', 'w')
        h, w = np.shape(array)
        line = ""
        # print("Thing:" + str(len(array[0])))
        for py in range(0, h):  # height
            for px in range(0, w):  # width
                maths = 0.1236 * math.tan((array[py][px] / 2842.5) + 1.1863)
                line += str(maths) + " "
            f.write(line + "\n\n")
            line = ""
        f.close()

        color_img = fromarray(color)
        color_img.save('test-color.png')
		depth_img = fromarray(depth)
		depth_img.save('test-depth.png')
		
        # cv2.imwrite('test.png', color)
        # time.sleep(1)

        # f.write(str(depth[240][320]) + "\n")

        # quit program when 'esc' key is pressed
