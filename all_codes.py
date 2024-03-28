#draw_circle

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('draw_circle')
    rospy.loginfo('node has been started')

    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 2.0
        msg.linear.y = 2.0
        msg.angular.z = 1.0
        pub.publish(msg)
        rate.sleep()



#draw square


 #!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rospy
from turtlesim.msg import Pose
import math
import time



def pose_callback(pose):
    global x
    global y
    global yaw 

    yaw = pose.theta
    x = pose.x
    y = pose.y



def move(speed,distance,is_forward):
    cmd = Twist()
    global x,y
    x0 = x
    y0 = y

    if(is_forward):
        cmd.linear.x = abs(speed)
    else:
        cmd.linear.x = -abs(speed)

    distance_moved = 0
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)      

    while True :
        pub.publish(cmd)
        rate.sleep()


        distance_moved = distance_moved + abs( math.sqrt(((x-x0)**2)+ ((y-y0)**2)))

        if not (distance_moved<distance):
            rospy.loginfo('reached')
            break
    cmd.linear.x = 0
    pub.publish(cmd)
    rotate(90,30,False)
    move(3.0,8.0,True)


def rotate(ang_velocity,rel_angle,clockwise):
    global yaw

    cmd = Twist()


    theta0 = yaw
    ang_speed = math.radians(abs(ang_velocity))

    if (clockwise):
        cmd.angular.z = -abs(ang_speed)
    else:
        cmd.angular.z = abs(ang_speed)

    ang_speed = 0.0
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    
    t0 = rospy.Time.now().to_sec()

    while True:
        pub.publish(cmd)
        

        t1 = rospy.Time.now().to_sec()
        current_angle = (t1-t0)* ang_speed
        rate.sleep()


        if (current_angle<rel_angle):
            break

    cmd.angular.z = 0.0
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    




def spiral(pose: Pose):
    global x
    global y

    cmd = Twist()

    count = 0
    vk = 1
    wk = 2
    rk = 0.5
    constant_velocity = 4
    rate = rospy.Rate(1)

    while (pose.x<9.0) and (pose.y<9.0):
        rk = rk + 0.5
        cmd.linear.x = rk
        cmd.angular.z = constant_velocity
        pub.publish(cmd)
        rate.sleep()
    cmd.linear.x = 0.0
    pub.publish(cmd)


def rotate90():
    cmd = Twist()
    cmd.angular.z = 2
    current_angle = 0
    t0 = rospy.Time.now().to_sec
    while current_angle< (math.pi):
        pub.publish(cmd)
        t1 = rospy.Time.now().to_sec
        current_angle = 2*(t1-t0)
        rate = rospy.Rate(200)
        rate.sleep()
    cmd.angular.z = 0
    pub.publish(cmd)    

def square():
    count = 0
    while count<4:
        #move(3.0,8.0,True)
        rotate90()
        count+=1


def go_to_goal(x_goal,y_goal):
    global x
    global y 
    cmd = Twist

    while(True):
        k_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x)**2)+ ((y_goal-y)**2)))

        linear_speed = distance*k_linear

        k_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y,x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*k_angular

        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        pub.publish(cmd)


        if (distance<0.01):
            break




        





if __name__ == '__main__':

    rospy.init_node('square')
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10) 
    sub = rospy.Subscriber("/turtle1/pose",Pose,callback= pose_callback)
    time.sleep(10)

    #rotate(90,30,False)
    #move(3.0,8.0,True)
    #spiral(Pose)
    #square()
    #rotate90()
    #go_to_goal(1.0,1.0)


    
#turlesim codes
    
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt   


class TurtleBot:
    
    def __init__(self):

        rospy.init_node('go_to_goal')
        self.pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.sub = rospy.Subscriber("/turtle1/pose",Pose,callback=self.pose_callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def pose_callback(self,data):

        self.pose = data
        self.pose.x = round(self.pose.x,4)
        self.pose.y = round(self.pose.y,4)

    
    def distance(self,goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
    
    def linear_vel(self,goal_pose,constant = 1.5):
        return constant * self.distance(goal_pose)
    
    def angle_traversed(self,goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    
    def angular_vel(self,goal_pose,constant = 6):
        return constant * (self.angle_traversed(goal_pose) - self.pose.theta)
    
    def move2goal(self):
        goal_pose = Pose()

        goal_pose.x = 1.0
        goal_pose.y = 1.0

        distance_tol = 0.5

        vel = Twist()

        while self.distance(goal_pose)>= distance_tol:
            vel.linear.x = self.linear_vel(goal_pose)
            vel.linear.y = 0
            vel.linear.z = 0

            vel.angular.x = 0
            vel.angular.y = 0
            vel.angular.z = self.angular_vel(goal_pose)

            self.pub.publish(vel)
            self.rate.sleep()
        
        vel.linear.x = 0
        vel.angular.z = 0 
        self.pub.publish(vel)

        rospy.spin()

if __name__ == '__main__':
        try:
            x= TurtleBot()
            x.move2goal()
        except rospy.ROSInterruptException:
            pass            
    



#random_bot




#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist 
import random

if __name__ == '__main__':
    rospy.init_node('Random_Motion')
    rospy.loginfo('Random motion has been started')
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rate = rospy.Rate(5)



    #l = [0.3,0.6,0.9,1,2,2.3,2.6,2.9,3] #linear velocity
    #l1 = [0.2,0.4,0.6,0.8,1,1.2,1.4,1.6]#angular velocity

    #v = random.choice(l)
    #w = random.choice(l1)

    while not rospy.is_shutdown():
        l = [0.3,0.6,0.9,1,1.3,1.5,2] #linear velocity
        l1 = [1.57,-1.57,0.5,0.3]#angular velocity
        l2 = [0.3,0.6,0.9,1,2,2.3,2.6,2.9,3]#y axis linear velocity
        

        v = random.choice(l)
        u = random.random()
        w = random.choice(l1)
        
        print(v,u,w)
        msg = Twist()
        msg.linear.x = v  
        msg.linear.y = u
        msg.angular.z = w

        pub.publish(msg)
        rate.sleep()





#spiral turtlesim bot

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time 

def rotate90():
    cmd = Twist()
    cmd.angular.z = 2
    current_angle = 0
    t0 = rospy.Time.now().to_sec
    while current_angle< (math.pi/2):
        pub.publish(cmd)
        t1 = rospy.Time.now().to_sec
        current_angle = (cmd.angular.z)*(t1-t0)
        rate = rospy.Rate(4)
        rate.sleep()
    cmd.angular.z = 0
    pub.publish(cmd)    


def spiral(pose: Pose):
    cmd = Twist()
    vk = 1
    wk = 2
    rk = 0.5
    constant_velocity = 4
    rate = rospy.Rate(1)

    while (pose.x<7.0) and (pose.y<7.0):
        rk = rk + 0.5
        cmd.linear.x = rk
        #cmd.angular.z = constant_velocity
        pub.publish(cmd)
        rate.sleep()
        rotate90()
        rate.sleep()

    cmd.linear.x = 0.0
    pub.publish(cmd) 


if __name__ == '__main__':

    rospy.init_node('square')
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10) 
    sub = rospy.Subscriber("/turtle1/pose",Pose,callback= spiral)
    rospy.spin()

    





        





    
        
   
    

   
    
    

    
          
