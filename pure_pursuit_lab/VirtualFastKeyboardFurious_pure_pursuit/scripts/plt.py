import matplotlib 
import matplotlib.pyplot as plt
import csv
import numpy as np

filename = '../waypoints/error.csv'

with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
max_abs_error = np.zeros(len(path_points))
error_abs = np.zeros(len(path_points))
for i in range(len(path_points)):
    max_abs_error[i] = path_points[i][1]
    error_abs[i] = path_points[i][2]
max_error = 0 
plot_max_abs_error = []
total_abs_error = []

for i in range(len(max_abs_error)):
    if max_abs_error[i]>max_error:
        plot_max_abs_error.append( max_abs_error[i])
        max_error = max_abs_error[i]
    else:
        plot_max_abs_error.append(max_error)

Total_abs_error_last = 0
for i in range(len(error_abs)):
    Total_abs_error_last += abs(error_abs[i])
    total_abs_error.append(Total_abs_error_last)

print(Total_abs_error_last)
plt.figure('Max Error')
plt.plot(plot_max_abs_error, label="max_abs_error", c='red')
plt.ylabel('Error (m)')
plt.legend()
plt.xlabel('Time Steps')
plt.savefig('Max_Error')

plt.figure("Total_Absolute_error")
plt.plot(total_abs_error,  label="total_abs_error", c='blue')
plt.ylabel('Error (m)')
plt.legend()
plt.xlabel('Time Steps')
plt.savefig('Total_Absolute_Error')
plt.show()



