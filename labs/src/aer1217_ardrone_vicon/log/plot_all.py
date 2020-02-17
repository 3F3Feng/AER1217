import matplotlib.pyplot as plt
import csv
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Initial var for command log
d_x = []
d_y = []
d_z = []
d_t = []
d_qx = []
d_qy = []
d_qz = []
d_qw = []
d_rx = []
d_ry = []
d_rz = []

# Initial var for vicon log
x = []
y = []
z = []
t = []
rx = []
ry = []
rz = []

# Initial var for cmd log
c_t = []
c_rx = []
c_ry = []
c_z_dot = []
c_yaw = []

n = 5
log = ['first', 'second', 'third', 'fourth', 'fifth']

# extract data from /desired_position.csv
with open(log[n - 1] + '/_slash_desired_position.csv') as logfile:
    data_c = csv.reader(logfile, delimiter=',')
    i = 0
    for row in data_c:
        if 1 < i:
            d_x.append(float(row[10]))
            d_y.append(float(row[11]))
            d_z.append(float(row[12]))
            d_t.append(float(row[0]) / 1000000000)

            # For misused euler desired rotation log
            d_rx.append(float(row[14]))
            d_ry.append(float(row[15]))
            d_rz.append(float(row[16]))
            # For quaternion desired rotation log
            # [rx_temp, ry_temp, rz_temp] = euler_from_quaternion([float(row[14]), float(row[15]),
            #                                                      float(row[16]), float(row[17])])
            # d_rx.append(rx_temp)
            # d_ry.append(ry_temp)
            # d_rz.append(rz_temp)
        i = i + 1
    print('Number of desired position sent:', i)

# Calculate time interval for desired position
dt0 = 0
dto = []
for dt in d_t:
    dto.append(dt - dt0)
    dt0 = dt

# Extract data from /vicon/ARDroneCarre/ARDroneCarre.csv
with open(log[n - 1] + '/_slash_vicon_slash_ARDroneCarre_slash_ARDroneCarre.csv') as logfile:
    data = csv.reader(logfile, delimiter=',')
    j = 0
    for row in data:
        if 1 < j:
            x.append(float(row[10]))
            y.append(float(row[11]))
            z.append(float(row[12]))
            t.append(float(row[0]) / 1000000000)

            # Desired position does not include rotation around x and y axel
            # rx.append(float(row[14]))
            # ry.append(float(row[15]))
            # rz.append(float(row[16]))

            # For quaternion desired rotation log
            [rx_temp, ry_temp, rz_temp] = euler_from_quaternion([float(row[14]), float(row[15]),
                                                                 float(row[16]), float(row[17])])
            rx.append(rx_temp)
            ry.append(ry_temp)
            rz.append(rz_temp)
        j = j + 1
    print('Number of vicon msg received:', j)

# Calculate time interval for vicon log
t0 = 0
to = []
for tt in t:
    to.append(tt - t0)
    t0 = tt

if n > 3:  # the first three log doesn't include /cmd_vel_RHC
    # Extract command from /cmd_vel_RHC
    with open(log[n - 1] + '/_slash_cmd_vel_RHC.csv') as logfile:
        data = csv.reader(logfile, delimiter=',')
        k = 0
        for row in data:
            if 1 < k:
                c_t.append(float(row[0]) / 1000000000)
                c_rx.append(float(row[2]))
                c_ry.append(float(row[3]))
                c_z_dot.append(float(row[4]))
                c_yaw.append(float(row[8]))
            k = k + 1
        print('Number of cmd msg received:', k)

plt.figure()
plt.plot(d_x, d_y)
plt.plot(x, y)
plt.axis('equal')
plt.show()

if n > 3:
    figure, ax = plt.subplots(3, 1, True)
    ax[0].plot(c_t, c_rx)
    ax[0].plot(t, rx)
    ax[1].plot(c_t, c_ry)
    ax[1].plot(t, ry)
    ax[2].plot(d_t, d_rz)
    ax[2].plot(t, rz)
    plt.show()

# Plot three axel separately
plt.figure(1)
plt.plot(d_t, d_x)
plt.plot(t, x)
# plt.axis('equal')
plt.show()

plt.figure(2)
plt.plot(d_t, d_y)
plt.plot(t, y)
# plt.axis('equal')
plt.show()

plt.figure(3)
plt.plot(d_t, d_z)
plt.plot(t, z)
# plt.axis('equal')
plt.show()
