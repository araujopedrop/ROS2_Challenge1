#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"
#include "my_robot_interfaces/msg/turtle.h"
#include "my_robot_interfaces/msg/turtles.hpp"
#include <ctime>

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

        timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&turtleSpawnerNode::spawn_new_turtle, this));
        timer_publisher_ = this-> create_wall_timer(std::chrono::seconds(1),std::bind(&turtleSpawnerNode::publish_alive_turtles,this));
        server_ = this->create_service<my_robot_interfaces::srv::KillTurtle>("/catch_turtle", std::bind(&turtleSpawnerNode::callback_service_killer,
                                                                                                        this,
                                                                                                        std::placeholders::_1,
                                                                                                        std::placeholders::_2));

        client_kill_service_ = this->create_client<turtlesim::srv::Kill>("/kill");

        // Create publisher. Publish in topic /alive_turtles
        //  publish a Turtle() array.
        publisher_alive_turtles_ = this->create_publisher<my_robot_interfaces::msg::Turtles>("/alive_turtles",10);

        RCLCPP_INFO(this->get_logger(), "turtle_spawner is up!");
    }

    void spawn_new_turtle()
    {
        spawn_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(&turtleSpawnerNode::spawn_service, this)));
    }

    void spawn_service()
    {

        RCLCPP_INFO(this->get_logger(), "Executing spawn service");

        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        float posX = random_number(0.0, 10.0);
        float posY = random_number(0.0, 10.0);

        // wait for service
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server");
        }

        /*
            float32 x
            float32 y
            float32 theta
            string name # Optional.  A unique name will be created and returned if this is empty
            ---
            string name
        */

        // generate msg
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        request->x = posX;
        request->y = posY;
        request->name = prefix_turtle + std::to_string(counter_turtle);
        counter_turtle++;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();

            if (response->name != "")
            {
                // c_str allows me to use a char[] from a string
                RCLCPP_INFO(this->get_logger(), "Turtle was created!: %s ", response->name.c_str());

                /*
                    string name
                    float64 x
                    float64 y
                    float64 theta
                */

                // Add new turtle to the alive_turtles_vector_
                my_robot_interfaces::msg::Turtle new_turtle = my_robot_interfaces::msg::Turtle();

                new_turtle.name = response->name;
                new_turtle.x = request->x;
                new_turtle.y = request->y;
                new_turtle.theta = 0.0;
                
                alive_turtles_vector_.push_back(new_turtle);
                
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Turtle was NOT created!: %s ", response->name.c_str());
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in server Spawn!");
        }
    }

    float random_number(float min, float max)
    {
        float random_number = 0.0;

        random_number = (rand() / (double)RAND_MAX) * (max - min) + min;

        RCLCPP_INFO(this->get_logger(), "random_number: %f", random_number);

        return random_number;
    }

    void callback_service_killer(my_robot_interfaces::srv::KillTurtle::Request::SharedPtr request,
                                 my_robot_interfaces::srv::KillTurtle::Response::SharedPtr response)
    {
        /*
            string turtle
            ---
            bool success
        */
        std::string turtle_name = request->turtle;

        kill_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(&turtleSpawnerNode::kill_turtle, this, request->turtle)));

        response->success = true;
    }

    void kill_turtle(std::string request_name)
    {

        while (!client_kill_service_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server...");
        }

        auto request_turtlesim = std::make_shared<turtlesim::srv::Kill::Request>();

        /*
        string name
        ---
        */

        request_turtlesim->name = request_name;

        auto future = client_kill_service_->async_send_request(request_turtlesim);

        try
        {
            auto response_ = future.get();
            RCLCPP_INFO(this->get_logger(), "Turtle catched!");
            for (int i = 0; i < (int) alive_turtles_vector_.size() ;i++) 
            {
                if (alive_turtles_vector_.at(i).name == request_name)
                {
                    alive_turtles_vector_.erase(alive_turtles_vector_.begin() + i);
                    publish_alive_turtles();
                }
                break;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in service");
        }
    }

    void publish_alive_turtles()
    {
        auto msg = my_robot_interfaces::msg::Turtles();

        msg.turtles = alive_turtles_vector_;
        publisher_alive_turtles_->publish(msg);

    }

private:
    int counter_turtle = 2;
    std::string prefix_turtle = "turtle";
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_publisher_;
    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_vector_;

    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_kill_service_;
    rclcpp::Service<my_robot_interfaces::srv::KillTurtle>::SharedPtr server_;

    rclcpp::Publisher<my_robot_interfaces::msg::Turtles>::SharedPtr publisher_alive_turtles_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
