import matplotlib.pyplot as plt
import csv

c_x = []
c_y = []
c_z = []
c_t = []

x = []
y = []
z = []
t = []

begin = 1 + 0
end = 1 + 3000
with open('_slash_desired_position.csv') as logfile:
    data_c = csv.reader(logfile, delimiter=',')
    i = 0
    for row in data_c:
        if 1 < i:
            c_x.append(float(row[10]))
            c_y.append(float(row[11]))
            c_z.append(float(row[12]))
            c_t.append(float(row[0])/1000000000)

        i = i + 1
    print(i)

with open('_slash_vicon_slash_ARDroneCarre_slash_ARDroneCarre.csv') as logfile:
    data = csv.reader(logfile, delimiter=',')
    j = 0
    for row in data:
        if 1 < j:
            x.append(float(row[10]))
            y.append(float(row[11]))
            z.append(float(row[12]))
            t.append(float(row[0])/1000000000)

        j = j + 1
    print(j)

print(i/j)
print(c_t[-1] - c_t[0], 1/(c_t[1] - c_t[0]))
print(t[-1] - t[0], 1/(t[1] - t[0]))
print((c_t[-1] - c_t[0]) - (t[-1] - t[0]))
plt.plot
plt.plot(c_x, c_y)
plt.plot(x, y)
plt.show()
