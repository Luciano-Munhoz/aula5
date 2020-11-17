import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math

# 1/(((2+0+1+7+0+1+3+1+0+6)+(2+0+1+6+0+0+1+1+0+0)+(2+0+1+7+0+0+6+0+7+7)+(2+0+1+8+0+1+9+0+2+4))/4);
periodo = 0.04494382022
estado = "estado_1"
velAng=0.5


kp = 1
ki = 0.5
kd = 0.5

old_error = 0
sumError = 0

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')


def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw
    

def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg


def timerCallBack(event):
    global old_error, sumError, estado, velAng
    
    setpoint = 0.5
    scan_len = len(scan.ranges)
    msg = Twist()
   
    if not(scan_len > 0):
        msg.angular.z = 0
        msg.linear.x = 0
        
 
    elif estado == 'estado_1':
        print('Ajustando rota')
       
        if min(scan.ranges[scan_len-5 : scan_len+5]) < 100:
           
            estado = 'estado_2'
            old_error = sumError = 0
            msg.angular.z = 0
        else:
            if min(scan.ranges[scan_len-15 : scan_len+15]) < 100:
                msg.angular.z = velAng*0.5
            else:
                msg.angular.z = velAng
            

    elif estado == 'estado_2':
 
    
        read = min(scan.ranges[scan_len-15 : scan_len+15])
        P=I=D=0
        if read < 100:
            error = -(setpoint - read)
            varError = (error-old_error)/periodo
            sumError+=error*periodo
            
            old_error = error   
            
            P = kp*error
            I = ki*sumError
            D = kd*varError
        
        control = P+I+D
        print(read, P, I, D)
        
        if control > 1:
            control = 1
        elif control < -1:
            control = -1
        
      
        msg.linear.x = control
        msg.angular.z = 0


        if read>100:
            estado = 'estado_1'
       
    
    
    pub.publish(msg)
    

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(periodo), timerCallBack)

rospy.spin()
