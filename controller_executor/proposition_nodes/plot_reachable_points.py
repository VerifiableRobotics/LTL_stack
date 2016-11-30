import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
import std_msgs.msg
import pickle
import getpass

class PlotPoints(object):
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.figure.canvas.draw()

        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

        plt.ion()     # turns on interactive mode
        plt.show()    # now this should be non-blocking

    def callback(self, data):
        self.ax.scatter(data.data[0],data.data[1],data.data[2])

    def plot_point(self,x,y,z):
        self.ax.scatter([x],[y],[z])
        self.ax.figure.canvas.draw()

    def save_plot(self):
        #pickle.dump(self.fig,file('points.pickle','w+'))
        self.fig.savefig('/home/{0}/points.png'.format(getpass.getuser()))   # save the figure to file
        plt.close(self.fig)    # close the figure


if __name__ == "__main__":
    #rospy.init_node('plot_points')
    #a = PlotPoints()
    #rospy.Subscriber("/plot_points", std_msgs.msg.UInt64MultiArray, callback=a.callback)
    #rospy.spin()

    a = PlotPoints()
    a.plot_point(0,0,0)
    a.plot_point(1,1,1)
    a.plot_point(2,2,2)
    a.save_plot()

    raw_input('Press Enter to continue...')
