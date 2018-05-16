import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

def read_datafile(file_name):
    # the skiprows keyword is for heading, but I don't know if trailing lines
    # can be specified
    data = np.loadtxt(file_name, delimiter=',', skiprows=10)
    return data

# data = read_datafile('test_edit.csv')
data = np.genfromtxt('test_roll.csv', delimiter=',', skip_header=10,
                     skip_footer=10, names=['x', 'y', 'z'])


fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("head orientation estimation")    
ax1.set_xlabel('time')
ax1.set_ylabel('angle(rad)')

ax1.plot(data['x'], c='r', label='roll')
ax1.plot(data['y'], c='g', label='pitch')
ax1.plot(data['z'], c='b', label='yaw')

leg = ax1.legend()

plt.show()
