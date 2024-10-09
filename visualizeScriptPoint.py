#from ompl visualize documentation w/ modifications

#Right now, works for point robot. Technically works for square, but not sure if 
#orientation is necessary to plot. Run by copy and pasting the path output into path.txt and 
#uncommenting the environment wanted.

#TODO: show the square robot correctly (add as rectangle object?)

import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patch

data = numpy.loadtxt('path.txt')
fig, ax = plt.subplots()
ax.plot(data[:, 0],data[:, 1],'.-')


#ENVIRONMENT 1:
"""#(x of lower left, y of lower left), width, height:
r1 = patch.Rectangle((1, 1), 2, 4)
r2 = patch.Rectangle((4, 4), 2, 1)
r3 = patch.Rectangle((4, 1), 2, 1)

ax.add_patch(r1)
ax.add_patch(r2)
ax.add_patch(r3)"""

#Environment 2:
r1 = patch.Rectangle((2,0), 2.5, 1)
r2 = patch.Rectangle((4.5, 1.5), 1, 5)
r3 = patch.Rectangle((3, 2), 2, 2)
r4 = patch.Rectangle((1, 3), 1, 3)

ax.add_patch(r1)
ax.add_patch(r2)
ax.add_patch(r3)
ax.add_patch(r4)

plt.show()