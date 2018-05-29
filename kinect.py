#import the necessary modules
import freenect
import cv2
import numpy as np
import sys
import time
from scipy.misc import imshow
from PIL.Image import fromarray
 
#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth(format = freenect.DEPTH_MM)
    return array
if __name__ == "__main__":
    f = open('middlepixel.txt','w')
    while 1:
        #get a frame from RGB camera
        #frame = get_video()
        #get a frame from depth sensor
        array = get_depth()
        depth = array.astype(np.uint8)
        
        #depth = depth.astype(np.uint16)
        #display cv2.imshow('RGB image',cv2.Canny(frame, 150, 200))
        #display depth image
        #cv2.imshow('Depth image',depth)
        imshow(array)
        f = open('imageoutput.txt', 'w')
        h, w = np.shape(array)
        line = ""
        #print("Thing:" + str(len(array[0])))
        for py in range(0,h):  # height
            for px in range(0,1):  # width
                line += str(array[py][px]) + " "
            f.write(line + "\n\n")
            line = ""
        f.close()
        
        img = fromarray(array)
        img.save('test2.tiff')
        #time.sleep(1)
        
        #f.write(str(depth[240][320]) + "\n")
 
        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    
    cv2.imwrite('test.png', array)
    cv2.destroyAllWindows()