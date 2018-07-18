from scipy import stats
import numpy as np
import sys

file_name = sys.argv[1]
print(file_name)
array = np.loadtxt(file_name)  # 20 by 20
array = array[array != 0]

indicies = np.arange(1, len(array) + 1)
#indicies = [1]

#print(indicies)
#print(array[:,10])
#print("Onwards!")

#print(len(indicies))
#print(len(array[:,10]))
#print("NOW Onwards!")

print(len(indicies))
print(len(array))

slope, intercept, r_value, p_value, std_err = stats.mstats.linregress(indicies, array)

print("Slope: " + str(slope))
print("Intercept: " + str(intercept))
print("R: " + str(r_value))
print("R Squared: " + str(r_value * r_value))
print("P: " + str(p_value))
print("Std Error: " + str(std_err))

i_threshold = 0.4
a_threshold = -4
b_threshold = 3000

if (r_value * r_value) > i_threshold and slope < a_threshold and  intercept > b_threshold:
    print("We're clear!")
else:
    print("Something's in the way!")
    