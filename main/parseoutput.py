#%%
import re

lines = []
with open("output2.txt") as f:
    flns = f.readlines()
    for line in flns:
        if len(line.strip()) > 4:
            lines.append(line)

print(lines[:3])

vals = []
for line in lines:
    if len(line) > 4:
        mstr = re.match("\[\d\] (\w+) \[\d\] (\w+) \[\d\] (\w+) \[\d\] (\w+)", line)
        if mstr:
            print(mstr.groups())
            vals.append(mstr.groups())

# print(vals[:4])

hex = []
for row in vals:
    hex.append(int(row[0] + row[1], 16))
    hex.append(int(row[2] + row[3], 16))

print(hex)

import matplotlib.pyplot as plt
plt.plot(hex)
plt.show()


# %%
