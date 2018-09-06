import freenect
import obstacle_detection
import actual_haptic
import debug_haptic
import time
import config

# Effect codes from the DRV2605's data sheet.
# https://learn.adafruit.com/assets/21843
close_vibration = 12  # Triple Click
medium_vibration = 10  # Double Click
far_vibration = 1  # Strong Click

DEBUG = False


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

result_counter = 0
far_results = 0
medium_results = 0
close_results = 0

def reset_counters():
    print("Counters reset!")
    global result_counter, far_results, medium_results, close_results
    result_counter = 0
    far_results = 0
    medium_results = 0
    close_results = 0

def print_counters():
    print("Result: {} Far: {} Medium: {} Close: {}".format(result_counter, far_results, medium_results, close_results))

if __name__ == "__main__":
    far = 0
    medium = 0
    close = 0
    if not DEBUG:
        actual_haptic = debug_haptic()
    else:
        actual_haptic.begin()
    while 1:
        print("Calling Camera")
        array = get_depth()
        print ("Camera image obtained!")
        # detection_result = obstacle_detection.change_this_name(array, 1)
        detection_result = obstacle_detection.detect(2, image = array)
        # We want the majority to decide what we beep, go with 3 for now?
        # If we're only doing 3, we can cheat and check if any category has 2. Otherwise go with medium? Or an error state?
        # Provide feedback based on distance
        if detection_result == -1:
            print("No object detected!")
            reset_counters()
            
        elif detection_result > 1:
            print("Object detected, {}mm away!".format(detection_result))
            result_counter += 1
            print_counters()
            if detection_result > 1500:
                far_results += 1
                if result_counter > 2 and far_results > 1:
                    actual_haptic.playEffect(far_vibration)
                    reset_counters()
                    far += 1
            elif detection_result > 1000:
                medium_results += 1
                if result_counter > 2 and medium_results > 1:
                    actual_haptic.playEffect(medium_vibration)
                    reset_counters()
                    medium += 1
            else:
                close_results += 1
                if result_counter > 2 and close_results > 1:
                    actual_haptic.playEffect(close_vibration)
                    reset_counters()
                    close += 1
            if result_counter == 3:
                print("Error state! 3 very different measurements!")
                reset_counters()
        # Limiting the amount of times we access the camera / buzzer
        print("Buzz stats: Far {} Med {} Close {}".format(far, medium, close))
        time.sleep(0.55)
