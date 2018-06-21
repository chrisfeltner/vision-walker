import test_detect


# For the Correlation coefficient, we want to step through a range
# using floats. This isn't a native thing in Python.
def frange(start, stop, step):
    x = start
    while x < stop:
        yield x
        x += step


# Open related files, then give Summary a header.
report = open('DetailedReport.txt', "w")
summary = open('Summary.txt', "w")
summary.write('TestNum I A B CorrectTrue CorrectFalse FalsePositives FalseNegatives Precision Accuracy P-Acc\n')

# Keeps track of which combination we're on.
counter = 0
# Stores the information from the answers text file
answers = []

# Open the answer file, then grab the first line (the # of test files)
answer_file = open('answers.txt', 'r')
test_count = int(answer_file.readline())

# Keep track of correct and incorrect #s for each test file
correctT_array = [0] * test_count
correctF_array = [0] * test_count
falsepos_array = [0] * test_count
falseneg_array = [0] * test_count

# Read in answers, split it away from the measured distance.
for answer in xrange(0, test_count):
    hasObstacle, distance = answer_file.readline().split(' ')
    answers.append((hasObstacle, distance))

# For loop over correlation coefficient, slope, and y-intercept
for i in frange(0.2, 0.8, 0.1):
    for a in xrange(-11, 0):
        for b in xrange(2000, 9000, 1000):
            # Store this combination's statistics for later writing
            counter += 1
            report.write('Test {} I:{} A:{} B:{}\n'.format(counter, i, a, b))
            correctT = 0
            correctF = 0
            false_pos = 0
            false_neg = 0
            for test in xrange(0, test_count):
                result = test_detect.detect(str(test + 1) + '.txt', i, a, b)
                if str(result) == answers[test][0]:
                    if result is True:
                        result_string = "Correct True"
                        correctT += 1
                        correctT_array[test] += 1
                    elif result is False:
                        result_string = "Correct False"
                        correctF += 1
                        correctF_array[test] += 1
                elif result is True:
                    result_string = "False Positive"
                    false_pos += 1
                    falsepos_array[test] += 1
                else:
                    result_string = "False Negative"
                    false_neg += 1
                    falseneg_array[test] += 1
                report.write('{}.txt Expected: {} Actual: {} Result: {}\n'.format(
                    str(test + 1), answers[test][0], result, result_string))
            report.write('\n')
            if correctF == 0 and false_neg == 0:
                precision = 1  # Not getting false negatives is good, I guess? It also didn't get any correct though...
            else:
                precision = correctF / (correctF + false_neg)
            accuracy = (correctF + correctT) / (test_count * 1.0)
            p_acc = (precision + accuracy) / 2.0
            summary.write('{} {} {} {} {} {} {} {} {} {} {}\n'.format(
                counter, i, a, b, correctT, correctF, false_pos, false_neg, precision, accuracy, p_acc))

# Write out each test file's stats.
report.write("File Results")
for i in xrange(0, 10):
    report.write("{}.txt Correct: {} False Positive: {} False Negative: {}".format(
        str(i + 1), correct_array[i], falsepos_array[i], falseneg_array[i]))
report.close()
summary.close()
