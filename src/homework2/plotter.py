import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

# Import the x and y coordinates saved from the simple mapper
df = pd.read_csv('robot_points.txt',names=['x','y'])
x = df.x.tolist()
y = df.y.tolist()

# Plot all the saved x and y coordinates from the read file
plt.plot(x,y,'bo')
plt.show()
