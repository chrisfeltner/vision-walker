# import the necessary modules
import freenect
import cv2
import numpy as np
import sys
import time
from scipy.misc import imshow
from PIL.Image import fromarray

def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array

# function to get depth image from kinect
def get_depth():
    print("Ayyy")
    array, _ = freenect.sync_get_depth(format=freenect.DEPTH_MM)
    print("Lmaooo")
    return array


if __name__ == "__main__":
    #f = open('middlepixel.txt', 'w')
    while 1:
        print("Hello?")
        # get a frame from RGB camera
        # frame = get_video()
        # get a frame from depth sensor
        array = get_depth()
        depth = array.astype(np.uint8)
        color = get_video()

        print("Hi?")
        # depth = depth.astype(np.uint16)
        # display cv2.imshow('RGB image',cv2.Canny(frame, 150, 200))
        # display depth image
        # cv2.imshow('Depth image',depth)
        imshow(array)
        f = open('imageoutput.txt', 'w')
        h, w = np.shape(array)
        line = ""
        # print("Thing:" + str(len(array[0])))
        for py in range(0, h):  # height
            for px in range(0, w):  # width
                line += str(array[py][px]) + " "
            f.write(line + "\n\n")
            line = ""
        f.close()

        img = fromarray(array)
        cv2.imwrite('test.png', color)
        # time.sleep(1)

        # f.write(str(depth[240][320]) + "\n")

        # quit program when 'esc' key is pressed
