import rospy
from std_msgs.msg import Float32



def yCallback(ydata):
    global init, xvalues, yvalues, y_offset, time
    print("y")
    y = ydata.data

    if not init:
        init = True
        y_offset = y
        time = rospy.Time.now()

    yvalues.append(y - y_offset)
    xvalues.append(rospy.Time.now().secs) 

    if rospy.Time.now() - time > rospy.Duration(4.0):
        y_sub.unregister()
        # plt.plot(xvalues, yvalues)
        # plt.axis([xvalues[0], xvalues[len(xvalues)-1], -1, 2])
        # plt.show()

        print(yvalues)
        
        return


# ---main---
rospy.init_node("line")

yvalues = []
xvalues = []

y_offset = 0.0

init = False
time = 0

global y_sub
y_sub = rospy.Subscriber("/ub05/y", Float32, yCallback, queue_size=1)