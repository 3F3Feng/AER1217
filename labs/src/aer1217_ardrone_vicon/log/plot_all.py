import matplotlib.pyplot as plt
import csv
from tf.transformations import euler_from_quaternion, quaternion_from_euler

c_x_1 = []
c_y_1 = []
c_z_1 = []
c_t_1 = []

x = []
y = []
z = []
t = []

n = 5
log = ['first', 'second', 'third', 'fourth', 'fifth']
with open(log[n - 1] + '/_slash_desired_position.csv') as logfile:
    data_c = csv.reader(logfile, delimiter=',')
    i = 0
    for row in data_c:
        if 1 < i:
            c_x_1.append(float(row[10]))
            c_y_1.append(float(row[11]))
            c_z_1.append(float(row[12]))
            c_t_1.append(float(row[0]) / 1000000000)

        i = i + 1
    print(i)

ct0 = 0
cto = []
for ct in c_t_1:
    cto.append(ct - ct0)
    ct0 = ct

with open(log[n - 1] + '/_slash_vicon_slash_ARDroneCarre_slash_ARDroneCarre.csv') as logfile:
    data = csv.reader(logfile, delimiter=',')
    j = 0
    for row in data:
        if 1 < j:
            x.append(float(row[10]))
            y.append(float(row[11]))
            z.append(float(row[12]))
            t.append(float(row[0]) / 1000000000)

        j = j + 1
    print(j)

t0 = 0
to = []
for tt in t:
    to.append(tt - t0)
    t0 = tt
print(i / j)
# print(c_t_1[-1] - c_t_1[0], 1 / (c_t_1[1] - c_t_1[0]))
# print(t[-1] - t[0], 1 / (t[1] - t[0]))
# print((c_t_1[-1] - c_t_1[0]) - (t[-1] - t[0]))
plt.figure()
plt.plot(c_t_1, c_x_1)
plt.plot(t, x)
# plt.axis('equal')
plt.show()

plt.figure()
plt.plot(c_t_1, c_y_1)
plt.plot(t, y)
# plt.axis('equal')
plt.show()

plt.figure()
plt.plot(c_t_1, c_z_1)
plt.plot(t, z)
# plt.axis('equal')
plt.show()

# plt.figure()
# plt.plot(cto[1:])
# plt.show()
#
# plt.figure()
# plt.plot(to[1:])
# plt.show()