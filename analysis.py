import test_detect


def frange(start, stop, step):
    x = start
    while x < stop:
        yield x
        x += step


report = open('DetailedReport.txt', "w")
summary = open('Summary.txt', "w")
summary.write('TestNum, I, A, B, Correct, FalsePositives, FalseNegatives\n')

counter = 0

answers = []

answer_file = open('answers.txt', 'r')
test_count = int(answer_file.readline())

for answer in xrange(0, test_count):
    hasObstacle, distance = answer_file.readline().split(' ')
    answers.append((hasObstacle, distance))
print(answers)

for i in frange(0.2, 0.8, 0.1):
    for a in xrange(-11, 0):
        for b in xrange(2000, 8250, 250):
            counter += 1
            report.write('Test {} I:{} A:{} B:{}\n'.format(counter, i, a, b))
            correct = 0
            false_pos = 0
            false_neg = 0
            for test in xrange(0, test_count):
                result = test_detect.detect(str(test + 1) + '.txt', i, a, b)
                if str(result) == answers[test][0]:
                    result_string = "Correct"
                    correct += 1
                elif result is True:
                    result_string = "False Positive"
                    false_pos += 1
                else:
                    result_string = "False Negative"
                    false_neg += 1
                report.write('{}.txt Expected: {} Actual: {} Result: {}\n'.format(
                    str(test + 1), answers[test][0], result, result_string))
            report.write('\n')
            summary.write('{} {} {} {} {} {} {}\n'.format(
                counter, i, a, b, correct, false_pos, false_neg))
report.close()
summary.close()
