import numpy as np
import matplotlib.pyplot as plt

l = 500000
wd = np.zeros(l, dtype=int)
ws = np.zeros(l)
bd = np.zeros(l, dtype=int)
for i in range(l):
    # wd[i] = np.random.randint(low=-179, high=180)
    ws[i] = np.random.random()*12
    # bd[i] = np.random.randint(low=-179, high=180)

fig1 = plt.figure()
plt.hist(wd)
plt.title("Wind Direction")
fig2 = plt.figure()
plt.hist(ws)
plt.title("Wind Speed")
fig3 = plt.figure()
plt.hist(bd)
plt.title("Boat bow Direction")
plt.show()