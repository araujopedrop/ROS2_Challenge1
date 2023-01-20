#include "rclcpp/rclcpp.hpp"
 
/*

    Functionality

    Run a control loop (for example using a timer with a high rate) to reach a given target point. The first turtle on the screen “turtle1” will be the “master” turtle to control. To control the turtle you can subscribe to /turtle1/pose and publish to /turtle1/cmd_vel.

    The control loop will use a simplified P controller.

    Subscribe to the /alive_turtles topic to get all current turtles with coordinates. From that info, select a turtle to target (to catch).

    When a turtle has been caught by the master turtle, call the service /catch_turtle advertised by the turtle_spawner node.

*/


class turtleControllerNode : public rclcpp::Node
{
public:
    turtleControllerNode() : Node("turtle_controller")
    {
        //Run control loop every n seconds (n is in miliseconds)
        // Create wall_timer
        // Create callback for wall timer
        //   See if I'm catching a turtle or not. If yes, call control algorithn

        //Create control algorithm
        // Define threshold
        // Define PID parameters
        // Subscribe to cmd_vel and set values using PID controller

        //Subscribe to /alive_turtles
        // Subscribe
        // Get first turtle or closest turtle
        
        //Advertise /catch_turtle service
        // If a turtle has been caught -> call service with the turtleName (string)
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}