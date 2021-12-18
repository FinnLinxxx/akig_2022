import rospy
import numpy as np
import matplotlib.pyplot as plt


np.set_printoptions(precision=4)
np.set_printoptions(suppress=True)

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

traj_ = np.array([0,0,0])
traj_ = np.vstack((traj_, traj_))

tachy_ = np.array([0,0,0])
tachy_ = np.vstack((tachy_, tachy_))

class Tachymeter(PointStamped):
    """
    inherits from rospy PointStamped Object
    Can be Positions given by eg. Tachymeter, Odometry or GPS
    additional Callback function for ros Subscriber
    """

    def callback(self, tachy_msg):
        global tachy_
        #rospy.loginfo("Received position [x]%5.2f  [y]%5.2f [z]%5.2f",
                      #tachy_msg.x, tachy_msg.y, tachy_msg.z)


        tachy_ = np.vstack((tachy_, np.array([(-1)*tachy_msg.point.x, tachy_msg.point.y, tachy_msg.point.z])))

class Husky_Odometry(Odometry):

    def callback(self, msg):
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
        #print(msg.header.stamp.secs)
        global traj_

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        #print("{0:.3f} {1:.3f} {2:.4f}".format(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))
        
        #self.set_xyr((traj_, np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]))

        traj_ = np.vstack((traj_, np.array([msg.pose.pose.position.x, (-1)*msg.pose.pose.position.y, yaw])))

    #def set_xyr(self, values):
        #self.point.x = value
        
                                   
if __name__ == '__main__':
    try:
        rospy.init_node('husky_odometry', anonymous=True)
        husky_odom_traj = Husky_Odometry()
        pos = Tachymeter()

        husky_odom_traj_sub = rospy.Subscriber('/odometry/filtered', Odometry, husky_odom_traj.callback)
        tachy_sub = rospy.Subscriber('/tachy_points', PointStamped, pos.callback)

        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot([-20, 20],[-20, 20], 'b-')
        line2, = ax.plot([-20, 20],[-20, 20], 'r-')
        while not rospy.is_shutdown():

            line1.set_xdata(traj_[:,1])
            line1.set_ydata(traj_[:,0])
            
            line2.set_xdata(tachy_[:,1])
            line2.set_ydata(tachy_[:,0])
            
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            
            rospy.sleep(0.1)
            


    except rospy.ROSInterruptException:
        pass
