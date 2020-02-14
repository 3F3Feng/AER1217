import matplotlib.pyplot as plt
import csv

c_x_1 = []
c_y_1 = []
c_z_1 = []
c_t_1 = []

x = []
y = []
z = []
t = []

n = 1
begin = 1 + 0
end = 1 + 3000
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

print(i / j)
print(c_t_1[-1] - c_t_1[0], 1 / (c_t_1[1] - c_t_1[0]))
print(t[-1] - t[0], 1 / (t[1] - t[0]))
print((c_t_1[-1] - c_t_1[0]) - (t[-1] - t[0]))
plt.figure()
plt.plot(c_x_1, c_y_1)
plt.plot(x, y)
plt.axis('equal')
plt.show()