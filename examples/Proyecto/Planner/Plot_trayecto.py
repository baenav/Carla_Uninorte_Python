

# import carla
import matplotlib.pyplot as plt 
# import matplotlib.axis as ax
plt.ion()
class Plot_trayecto(object):



    def __init__(self):

        self.x_history = []
        self.y_history = []
        self.vehicles_x = []
        self.vehicles_y = []
        print("Showing trajectory map")
        self.background = plt.imread("map_view5.png")

    # def load_graph(self):
    #     fig = self.fig
    #     plt.plot()
    #     # plt.axis(-200, 200, -200, 200)
    #     plt.xlim((-400, 400))  
    #     plt.ylim((-400, 400))  
    #     plt.show()
    
    # def update_player_loc(self,location):

    #     xloc = location[0]
    #     yloc = location[1]
        
    #     fig = self.fig
    #     plt.plot(xloc, yloc)
    #     plt.show()
    #     print("printing location")

    def start_plot(self):
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([],[], '-')
        self.other_lines, = self.ax.plot([],[], '.')
        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        # self.ax.set_position([10, 10, 10, 10], which='active')
        self.ax.set_xlim(-400, 400)
        self.ax.set_ylim(-400, 400)
        # self.ax.imshow(self.background, extent = [-300,375, -400, 375])
        self.ax.imshow(self.background, extent=[-329,329, -462,462])
        #Other stuff
        self.ax.grid()

    def update_plot(self,x_value, y_value, vehicles_x_pos, vehicles_y_pos):

        #Update data (with the new _and_ the old points)
        x_offset = 40
        y_offset = -28
        self.x_history.append(-x_value+x_offset)
        self.y_history.append(y_value+y_offset)

        self.lines.set_xdata(self.x_history)
        self.lines.set_ydata(self.y_history)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #We need to draw *and* flush
        self.figure.canvas.draw()

        if len(vehicles_x_pos) != 0:
            for i in range(len(vehicles_x_pos)):
                self.vehicles_x.append(-vehicles_x_pos[i]+x_offset)
                self.vehicles_y.append(vehicles_y_pos[i]+y_offset)
            
            self.other_lines.set_xdata(self.vehicles_x)
            self.other_lines.set_ydata(self.vehicles_y)
            self.figure.canvas.draw()
            self.vehicles_x = []
            self.vehicles_y = []
                

        self.figure.canvas.flush_events()



