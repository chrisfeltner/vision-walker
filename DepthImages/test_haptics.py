import freenect
import obstacle_detection
import debug_buzzer
import time
import config

# Effect codes from the DRV2605's data sheet.
# https://learn.adafruit.com/assets/21843
close_vibration = 12  # Triple Click
medium_vibration = 10  # Double Click
far_vibration = 1  # Strong Click

if not config.DEBUG:
    import example_buzzer


def get_depth():
    array, _ = freenect.sync_get_depth(format=freenect.DEPTH_MM)
    return array


'''
if __name__ == "__main__":
    if not debug:
        vibration_controller = example_buzzer.haptic()
    else:
        vibration_controller = debug_buzzer.debug_haptic()
    while 1:
        detection_result = obstacle_detection.change_this_name("test1.txt", 20)

        # Provide feedback based on distance
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
        # Limiting the amount of times we access the camera / buzzer
        time.sleep(0.5)
'''


if __name__ == "__main__":
    if not config.DEBUG:
        vibration_controller = example_buzzer.haptic()
    else:
        vibration_controller = debug_buzzer.debug_haptic()
    while 1:
        array = get_depth()
        detection_result = obstacle_detection.detect(2, image=array)

        # Provide feedback based on distance
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
        # Limiting the amount of times we access the camera / buzzer
        time.sleep(0.5)
