#!/usr/bin/env python

# --- imports ---
import rospy
import math
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# --- definitions ---
def resetGrid():
    global occupancy_grid
    
    # set all values to "FREE"
    occupancy_grid.data = [-1 for i in range(occupancy_grid.info.height * occupancy_grid.info.width)]

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def setCell(x,y,value):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = ((x - occupancy_grid.info.origin.position.x) * 1.0 / res)
    y_scaled = ((y - occupancy_grid.info.origin.position.y) * 1.0 / res)

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = value

def raytraceFree(ox, oy, tx, ty):
    global occupancy_grid

    dx = tx - ox
    dy = ty - oy
    dist = math.sqrt( dx*dx + dy*dy )
    
    inc = dist / occupancy_grid.info.resolution

    for i in range( int(inc) ):
        x = ox + dx * (i / inc)
        y = oy + dy * (i / inc)
        setCell(x, y, 0)

def computePoint(range, angle):
    x = range * math.sin(angle) #+ trans[1] 
    y = range * math.cos(angle) #+ trans[0]
    return (x, y)

def updateGrid(scan_msg):
    global occupancy_grid

    angle = scan_msg.angle_min
    inc = scan_msg.angle_increment

    #(trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
    #print(trans)
    #print(rot)

    #print(trans)

    #(r, p, y) = tf.transformations.euler_from_quaternion(rot)
    
    #angle +=  math.pi
    counter = 0
    for r in scan_msg.ranges:
        (x, y) = computePoint(r, angle)
        
        angle += inc
        counter += 1
        #print(angle)
        # if not (angle < -3 * math.pi / 4.0 or angle > 3 * math.pi / 4.0):
        #     continue
        if r > scan_msg.range_max or r < scan_msg.range_min:
            continue
        #raytraceFree(trans[1], trans[0], x , y)
        raytraceFree(0, 0, x , y)
        setCell(x, y, 100)
        


def scanCallback(scan_msg):

    global occupancy_grid
    global last

    if rospy.Time.now() - last < rospy.Duration(1.0):
        return

    last = rospy.Time.now()

    resetGrid()

    # convert scan measurements into an occupancy grid
    updateGrid(scan_msg)

    pub_grid.publish(occupancy_grid)


# --- main ---
rospy.init_node("scan_gridd")
last = rospy.Time.now()

# tranform listener
listener = tf.TransformListener()

# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = 0.03 # in m/cell

# width x height cells
occupancy_grid.info.width = 400
occupancy_grid.info.height = 400

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = -4
occupancy_grid.info.origin.position.y = -4
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)

rospy.spin()
