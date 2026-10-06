import numpy as np
import matplotlib.pyplot as plt
d = np.loadtxt("workspace.csv",delimiter =",")
print(d.shape)
plt.scatter(d[:,0],d[:,1], s=3)
plt.plot(0,0, "ks", markersize = 8)
plt.axis("equal")
plt.grid()
plt.xlabel("x(m)")
plt.ylabel("y(m)")
plt.title("2-link workspace")
plt.savefig("images/workspace.png",dpi =120)
plt.show()