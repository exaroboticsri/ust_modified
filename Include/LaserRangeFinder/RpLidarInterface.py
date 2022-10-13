import numpy as np
import matplotlib.pyplot as plt
from rplidar import RPLidar


lidar = RPLidar('COM34')
lidar.stop_motor()

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

# for i, scan in enumerate(lidar.iter_scans()):
#     print('%d: Got %d measurments' % (i, len(scan)))
#     if i > 10:
#         break

# lidar.stop()

# lidar.disconnect()


# def get_data():
#     lidar = RPLidar('COM34', baudrate=115200)
#     for scan in lidar.iter_scans(max_buf_meas=500):
#         break
#     lidar.stop()
#     lidar.stop_motor()
#     return scan
#
# for i in range(1000000):
#     if(i%7==0):
#         x = []
#         y = []
#     print(i)
#     current_data=get_data()
#     for point in current_data:
#         if point[0]==15:
#             x.append(point[2]*np.sin(np.deg2rad(point[1])))
#             y.append(point[2]*np.cos(np.deg2rad(point[1])))
#     plt.clf()
#     plt.scatter(x, y)
#     plt.pause(.1)
# plt.show()

