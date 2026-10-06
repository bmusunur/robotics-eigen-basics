import matplotlib.pyplot as plt
x = [0.0 , 0.3 , 0.6]
y = [0.0, 0.4, 0.0]
plt.scatter(x,y,s=80,color ='red')
plt.plot(x,y)
plt.axis("equal")
plt.xlabel("x")
plt.ylabel("y")
plt.title("2-link workspace")
plt.show()