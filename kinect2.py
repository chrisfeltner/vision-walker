import freenect
import detect2
import time
import example_buzzer
import numpy as np

# Effect codes from the DRV2605's data sheet.
# https://learn.adafruit.com/assets/21843
close_vibration = 12  # Triple Click
medium_vibration = 10  # Double Click
far_vibration = 1  # Strong Click


def get_depth():
    array, _ = freenect.sync_get_depth(format=freenect.DEPTH_MM)
    return array


if __name__ == "__main__":
    vibration_controller = haptic()
    while 1:
        array = get_depth()
        detection_result = detect2.detect(array, 2, 2000)

        if detection_result == -1:
            print("No object detected!")
        elif detection_result > 1:
            print("Object detected, {}mm away!".format(detection_result))
            if detection_result > 1500:
                vibration_controller.playEffect(far_vibration)
            elif detection_result > 1000:
                vibration_controller.playEffect(medium_vibration)
            else:
                vibration_controller.playEffect(close_vibration)
        time.sleep(0.5)
