import detect2

answer_file = open('answers.txt', 'r')
test_count = int(answer_file.readline())

answers = []

for answer in xrange(0, test_count):
    hasObstacle, distance = answer_file.readline().split(' ')
    answers.append((hasObstacle, distance))

# print(answers)

for test in xrange(0, test_count):
                result_string = ""
                result = detect2.detect_file(str(test + 1) + '.txt', 2, 2500)
                if result != -1 and result <= 2500:
                    if answers[test][0] == 'True':
                        result_string = "Correct True"
                    elif answers[test][0] == 'False':
                        result_string = "Incorrect True"
                else:
                    if answers[test][0] == 'True':
                        result_string = "Incorrect False"
                    elif answers[test][0] == 'False':
                        result_string = "Correct False"
                print('{}.txt Expected: {} Actual: {} Result: {}\n'.format(
                    str(test + 1), answers[test][1], result, result_string))
