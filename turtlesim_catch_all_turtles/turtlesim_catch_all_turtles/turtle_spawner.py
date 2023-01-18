#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from random import randint
from random import random
from functools import partial
from math import pi
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import Turtles
from my_robot_interfaces.srv import KillTurtle


class turtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")

        # Parameters
        self.declare_parameter("spawn_frequency", 1)
        self.declare_parameter("turtle_name_prefix", "turtle")

        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix_ = str(self.get_parameter(
            "turtle_name_prefix").value)
        self.use_sim_time_ = self.get_parameter("use_sim_time").value

        #Publishers, subscribers and services
        self.service_server_kill_turtle_ = self.create_service(
            KillTurtle, "/catch_turtle", self.turtle_killer)
        self.publisher_ = self.create_publisher(Turtles, "/alive_turtles", 10)
        self.publisher_timer = self.create_timer(1, self.callback_publisher)
        self.spawn_timer_ = self.create_timer(
            1/self.spawn_frequency_, self.turtle_spawner)

        self.turtle_number_ = 2

        # Array of Turtles objects
        self.turtles_alive_ = []

        self.turtle_spawner()
        self.get_logger().info("turtleSpawner is up!")

    # --------------------- Spawning methods ---------------------
    def turtle_spawner(self):

        self.spawn_service_client_ = self.create_client(Spawn, "/spawn")
        while not self.spawn_service_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server /spawn...")

        x_ = round(random()*(10-1)+1, 2)
        y_ = round(random()*(10-1)+1, 2)
        theta_ = round(random()*2*pi, 2)
        name_ = self.turtle_name_prefix_ + str(self.turtle_number_)

        self.turtle_number_ += 1

        msg = Spawn.Request()
        '''
            float32 x
            float32 y
            float32 theta
            string name # Optional.  A unique name will be created and returned if this is empty
            ---
            string name
        '''

        msg.x = x_
        msg.y = y_
        msg.theta = theta_
        msg.name = name_

        future = self.spawn_service_client_.call_async(msg)

        future.add_done_callback(
            partial(self.callback_spawner, x=x_, y=y_, theta=theta_))

    def callback_spawner(self, future, x, y, theta):
        # Appending new turtle to turtles_alive_
        try:
            if future.result().name != "":
                new_turtle = Turtle()
                new_turtle.name = future.result().name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta

                self.turtles_alive_.append(new_turtle)
            else:
                # if name is repeated -> name = ""
                self.get_logger().error("Turtle not created. Name repeated!")
        except:
            self.get_logger().error("Turtle not created. Error calling Spawn service!")



    # --------------------- Killing methods ---------------------
    def turtle_killer(self, request, response):
        #
        self.killer_service_client_ = self.create_client(Kill, "/kill")

        while not self.killer_service_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /kill service...")

        '''
        string name
        ---
        '''

        turtle_name = request.turtle

        if turtle_name != "":

            for turtle_robot in self.turtles_alive_:
                if turtle_robot.name == turtle_name:

                    msg = Kill.Request()
                    msg.name = turtle_name

                    self.get_logger().warn("Turtle to kill: " + str(turtle_name))

                    future = self.killer_service_client_.call_async(msg)

                    future.add_done_callback(
                        partial(self.callback_killer, turtleToKill=turtle_name))

                    response.success = True

                    break

            else:

                self.get_logger().warn("Turtle to kill already killed: " + str(turtle_name))
                response.success = False

        else:
            self.get_logger().error("Error")
            response.success = False

        return response

    def callback_killer(self, future, turtleToKill):
        self.get_logger().info(turtleToKill + " was killed!")

        # Remove turtle from turtles_alive_ list
        self.remove_turtle_from_list(turtleToKill)

        # Publish actualized turtles_alive_
        self.callback_publisher()

    def get_a_turtle(self):

        if self.turtles_alive_:
            turtle = self.turtles_alive_[0]
            if turtle.name:
                return turtle
            else:
                self.get_logger().error("There are no turtles alive!!!")
                return ""
        else:
            return ""

    def remove_turtle_from_list(self, turtle):
        for turtle_list in self.turtles_alive_:
            if turtle_list.name == turtle:
                self.turtles_alive_.remove(turtle_list)
                break

    # --------------------- Publisher methods ---------------------

    def callback_publisher(self):
        msg = Turtles()
        msg.turtles = self.turtles_alive_
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = turtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
