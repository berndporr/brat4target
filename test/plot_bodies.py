import numpy as np
import matplotlib.pyplot as plt
import sys

df = np.loadtxt(sys.argv[1])

# 2. Create the scatter plot
plt.scatter(df[:,0], df[:,1], s=np.sqrt(df[:,2])*1000, edgecolors='w')

# 3. Add titles and labels for clarity
plt.title('World objects')
plt.xlabel('X/m')
plt.ylabel('Y/m')

# 4. Display the plot
plt.show()
