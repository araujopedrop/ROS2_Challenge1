#!/usr/bin/python3

import rclpy
from math import sqrt,atan2,pi
from rclpy.node import Node
from turtlesim.msg import Pose
from functools import partial
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import Turtles
from my_robot_interfaces.srv import KillTurtle

'''
    Functionality:
    - Run a control loop to reach a given target point. 
      The first turtle on the screen "turtle1" will be the "master" turtle control. 
    - The control loop will a P controller
    - Subscribe to the /alive_turtles topic to get all current turtles with coordinates.
      From that info, select a turtle
    - When a turtle has been caught by the master turtle, call the service /catch_turtle advertised by
      the turtle_spawner node

'''

class turtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller") 
        
        #Control parameters
        self.T = 0.01
        self.P = 3
        self.P_ang = 6
        self.I = 0
        self.D = 0

        #Publishers, subscribers and services
        self.timer_controller_ = self.create_timer(self.T,self.control)
        self.subscriber_pose_  = self.create_subscription(Pose,"/turtle1/pose",self.callback_get_current_pos,10)
        self.alive_turtles_    = self.create_subscription(Turtles,"/alive_turtles",self.getTurtle,10)
        self.publisher_cmdVel_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.service_client_   = self.create_client(KillTurtle,"/catch_turtle")
        
        #Parameters
        self.declare_parameter("catch_closest_turtle_first",True)
        self.closest_turtle_ = self.get_parameter("catch_closest_turtle_first").value
        self.use_sim_time = self.get_parameter("use_sim_time").value


        #Other control parameters
        self.current_Pos_X_ = 0
        self.current_Pos_Y_ = 0
        self.current_Pos_angTheta_ = 0

        self.catchingATurtle_ = False
        self.turtleList_ = []

        self.turtleNameToCatch_ = ""
        self.setPointX_ = 0
        self.setPointY_ = 0
        self.setPointTheta_ = 0


        self.difX = 0.0
        self.difY = 0.0
        self.target_ang = 0.0
        self.target_ang_raw = 0.0
        self.eTheta = 0.0
        self.eDist = 0.0
        self.error_anterior_ = 0

        self.threshold_linear_ = 0.05
        self.threshold_angular_ = 0.25
        self.threshold_angular_2_ = 10

        self.turtlesKilled_ = []

        #For showing PIDs variables in cmd
        self.flag_debug = True
        
        self.current_time_ = self.get_current_time()

        self.get_logger().info("turtleControllerNode is up! ...")


    def control(self):
        #Control loop

        if self.catchingATurtle_ == False:
            #If I'm not catching a turtle -> get a turtle to catch
            

            #Choose if catch by closest turtle or by spawing order
            if self.closest_turtle_ == True:
                turtle = self.get_closest_turtle()
            else:
                turtle = self.get_first_turtle()

            #If a turtle is founded -> Get data from turtle
            if turtle.name != "":

                self.turtleNameToCatch_ = turtle.name
                self.setPointX_ = turtle.x
                self.setPointY_ = turtle.y
                self.setPointZ_ = turtle.theta

                self.catchingATurtle_ = True
                
            else:

                self.setPointX_ = 0.0
                self.setPointY_ = 0.0
                self.setPointZ_ = 0.0
                self.difX = 0.0
                self.difY = 0.0
                self.eDist = 0.0
                self.eTheta = 0.0

        else:
            #If I'm catching a turtle -> continue doing control to catch a turtle

            self.catchingATurtle_ = True

            #Control algorithm
            self.control_algorithm2()

            #For visualizing info
            if self.flag_debug:
                if abs(self.eTheta)  > abs(self.error_anterior_):
                    self.get_logger().error("--------------------------------------------------------")
                self.get_logger().info("Error dist: " + str(self.eDist))
                self.get_logger().info("difX: " + str(self.difX))
                self.get_logger().info("difY: " + str(self.difY))
                self.get_logger().info("target_ang: " + str(self.target_ang))
                self.get_logger().info("current_Pos_angTheta_: " + str(self.current_Pos_angTheta_))
                self.get_logger().info("Error theta: " + str(self.eTheta))
                self.error_anterior_ = self.eTheta





    def control_algorithm(self):

        self.get_error_signals()

        #Make msg for command velocity
        msg = Twist()

        if abs(self.eDist) < self.threshold_linear_ and abs(self.eTheta) < self.threshold_angular_:
            #If I'm close enough -> turtle catched

            self.stop_and_kill(msg)

        else:
            
            if abs(self.eDist) > self.threshold_linear_: 
                msg.linear.x = self.P  * self.eDist
            else:
                msg.linear.x = 0.0

            if abs(self.eTheta) > self.threshold_angular_: 
                msg.angular.z = self.P_ang * self.eTheta
            else: 
                msg.angular.z = 0.0


        self.publisher_cmdVel_.publish(msg)




    def control_algorithm2(self):

            self.get_error_signals()

            #Make msg for command velocity
            msg = Twist()

            if abs(self.eDist) < self.threshold_linear_ and abs(self.eTheta) < self.threshold_angular_:
                #If I'm close enough -> turtle catched

                self.stop_and_kill(msg)

            else:
                
                

                if abs(self.eTheta) > self.threshold_angular_2_: 
                    msg.linear.x = 0.0
                    msg.angular.z = self.P_ang * self.eTheta
                else: 
                    if abs(self.eTheta) > self.threshold_angular_: 
                        msg.angular.z = self.P_ang * self.eTheta
                        if abs(self.eDist) > self.threshold_linear_: 
                            msg.linear.x = self.P  * self.eDist
                        else:
                            msg.linear.x = 0.0
                    else:
                        if abs(self.eDist) > self.threshold_linear_: 
                            msg.linear.x = self.P  * self.eDist
                        else:
                            msg.linear.x = 0.0
                        msg.angular.z = 0.0

            self.publisher_cmdVel_.publish(msg)


    def get_error_signals(self):

        self.difX = self.setPointX_ - self.current_Pos_X_
        self.difY = self.setPointY_ - self.current_Pos_Y_

        self.eDist  = sqrt((self.difX)**2 + (self.difY)**2)
        self.target_ang = atan2(self.difY,self.difX)

        self.eTheta =  self.target_ang - self.current_Pos_angTheta_

        if self.eTheta > pi:
            self.eTheta -= 2*pi
        elif self.eTheta < -pi:
            self.target_ang += 2*pi
        
        
        

    def stop_and_kill(self,msg):
        #If I'm close enough -> turtle catched

        #Creating the catch message
        msg_turtle_kill = KillTurtle.Request()
        turtle_catched = Turtle()
        turtle_catched.name = str(self.turtleNameToCatch_)
        turtle_catched.x = self.setPointX_
        turtle_catched.y = self.setPointY_
        turtle_catched.theta = float(self.setPointTheta_)

        msg_turtle_kill.turtle = turtle_catched.name

        #Sending string Turtle name
        future = self.service_client_ .call_async(msg_turtle_kill)
        future.add_done_callback(partial(self.callback_turtle_killed,turtle_name=turtle_catched.name))

        #Tracking turtles catched
        self.turtlesKilled_.append(turtle_catched.name)

        #Stopping turtle
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.catchingATurtle_ = False





    def get_current_time(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs/1000000000.0 

    def callback_get_current_pos(self,msg):
        self.current_Pos_X_ = msg.x
        self.current_Pos_Y_ = msg.y
        self.current_Pos_angTheta_ = msg.theta


    def getTurtle(self,msg):
        self.turtleList_ = msg.turtles

    def get_first_turtle(self):
        #Returns the first turtle founded

        turtleToCatch = Turtle()
        for turtle in self.turtleList_:
            if turtle.name not in self.turtlesKilled_:
                turtleToCatch = turtle
        return turtleToCatch

    def get_closest_turtle(self):
        #Returns the closest turtle founded

        minimumDistance = 99999
        turtleToCatch = Turtle()
        for turtle in self.turtleList_:
            if turtle.name not in self.turtlesKilled_:
                distance = sqrt((self.current_Pos_X_ - turtle.x)**2 + (self.current_Pos_Y_ - turtle.y)**2)
                if distance < minimumDistance:
                    minimumDistance = distance
                    turtleToCatch = turtle
    
        self.get_logger().info("Turtle to catch: " + str(turtleToCatch.name))

        return turtleToCatch

    def callback_turtle_killed(self,future,turtle_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Turtle catched: " + str(turtle_name))
                self.turtleList_ = []
            else:
                self.get_logger().warn("Turtle NOT catched: " + str(turtle_name))
        except:
            self.get_logger().error("Turtle NOT catched. Error in service!")


 
 
def main(args=None):
    rclpy.init(args=args)
    node = turtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()