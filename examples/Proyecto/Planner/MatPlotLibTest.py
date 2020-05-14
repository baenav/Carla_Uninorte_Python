"""
Matplotlib Animation Example

author: Jake Vanderplas
email: vanderplas@astro.washington.edu
website: http://jakevdp.github.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import axis as ax
from Plot_trayecto import Plot_trayecto 

plotter = Plot_trayecto()

plotter.start_plot()
location = [0,0]
empty_list1 = []
empty_list2= []

while True:

    plotter.update_plot(location[0], location[1], empty_list1, empty_list2)

# xdata = [-100, 100]
# ydata = [-100,100]

# fig = plt.figure(num=1)
# plt.plot()
# # plt.axis(-200, 200, -200, 200)
# plt.xlim((-200, 200))  
# plt.ylim((-200, 200))  
# plt.show()