#include "rclcpp/rclcpp.hpp"
 

/*

    Functionality:

    Call the /spawn service to create a new turtle (choose random coordinates between 0.0 and 11.0 for both x and y), 
    and call the /kill service to remove a turtle from the screen. Both those services are already advertised by the turtlesim_node.

    Publish the list of currently alive turtles with coordinates on a topic /alive_turtles.

    Handle a service server to “catch” a turtle, which means to call the /kill service and remove the turtle from the array of alive turtles.

*/

class turtleSpawnerNode : public rclcpp::Node
{
public:
    turtleSpawnerNode() : Node("turtle_spawner")
    {
        //Call spawn service every n seconds with random numbers
        // generate random numbers between 0.0 and 11.0
        // turtlesim.srv import Spawn

        //Creates service to "catch" a turtle
        // receive a turtleName (string)

        //Call kill service when the turtle_controller tells me
        // turtlesim.srv import Kill

        //Create publisher. Publish in topic /alive_turtles
        // publish a Turtle() array. 

        
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}