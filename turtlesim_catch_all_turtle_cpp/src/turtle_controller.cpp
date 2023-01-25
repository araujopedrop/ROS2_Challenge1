#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtles.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"
#include "turtlesim/msg/pose.hpp"

#include <cmath>

/*

    Functionality

    Run a control loop (for example using a timer with a high rate) to reach a given target point. The first turtle on the screen “turtle1” will be the “master” turtle to control. To control the turtle you can subscribe to /turtle1/pose and publish to /turtle1/cmd_vel.

    The control loop will use a simplified P controller.

    Subscribe to the /alive_turtles topic to get all current turtles with coordinates. From that info, select a turtle to target (to catch).

    When a turtle has been caught by the master turtle, call the service /catch_turtle advertised by the turtle_spawner node.

    Parameters:

    catch_closest_turtle_first

    use_sim_time

*/

class turtleControllerNode : public rclcpp::Node
{
public:
    turtleControllerNode() : Node("turtle_controller")
    {


        
        this->declare_parameter("catch_closest_turtle_first", true);

        catch_closest_turtle_first_ = this->get_parameter("catch_closest_turtle_first").as_bool();

        subscriber_alive_turtles_ = this->create_subscription<my_robot_interfaces::msg::Turtles>("/alive_turtles",
                                                                                                 10,
                                                                                                 std::bind(&turtleControllerNode::callback_alive_turtles,
                                                                                                           this,
                                                                                                           std::placeholders::_1));

        subscriber_turtle_pose_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",
                                                                                  10,
                                                                                  std::bind(&turtleControllerNode::callback_get_pose,
                                                                                            this, 
                                                                                            std::placeholders::_1));


        publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);

        service_client_catch_turtle_ = this->create_client<my_robot_interfaces::srv::KillTurtle>("/catch_turtle");

        control_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)T*1000),std::bind(&turtleControllerNode::control_loop,this));

        catching_a_turtle_ = false;


        RCLCPP_INFO(this->get_logger(),"turtle_controller is up!");

        // Create control algorithm
        //  Define threshold
        //  Define PID parameters
        //  Subscribe to cmd_vel and set values using PID controller

    }

    void control_loop()
    {

        if (catching_a_turtle_ == true)
        {
            control_algorithm();
        }
        else
        {
            publish_velocity(0.0,0.0);

            try
            {
                turtle_to_catch_ = get_turtle();
                if (turtle_to_catch_.name != "")
                {
                    catching_a_turtle_ = true;

                    set_point_X_ = turtle_to_catch_.x;
                    set_point_Y_ = turtle_to_catch_.y;
                    
                    ePos = 0.0;
                    eTheta = 0.0;

                    difX = 0.0;
                    difY = 0.0;

                    RCLCPP_INFO(this->get_logger(),"Turtle to catch: %s", turtle_to_catch_.name.c_str());
                }
            }
            catch(const std::exception& e)
            {
                set_point_X_ = 0.0;
                set_point_Y_ = 0.0;
                ePos = 0.0;
                eTheta = 0.0;
                difX = 0.0;
                difY = 0.0;

                RCLCPP_WARN(this->get_logger(),"No turtle available!");
            }
            
        }

    }

    void control_algorithm()
    {
        difX = set_point_X_ - posX_;
        difY = set_point_Y_ - posY_;

        ePos = sqrt(pow(difX,2) + pow(difY,2));
        eTheta = atan2(difY,difX);

        if (eTheta > M_PI)
            eTheta -= 2*M_PI;
        else
        {
            if (eTheta < -M_PI)
                eTheta += 2*M_PI;
        }

        if (abs(eTheta) > abs(eTheta_threshold2))
            publish_velocity(0,P_ang*eTheta);
        else
        {
            if (abs(eTheta) > abs(eTheta_threshold))
            {
                if (abs(ePos) > abs(ePos_threshold))
                    publish_velocity(P*ePos,P_ang*eTheta);
                
            }
            else
            {
                if (abs(ePos) > abs(ePos_threshold))
                    publish_velocity(P*ePos,0.0);
                else
                {
                    //Turtle catched!
                    publish_velocity(0.0,0.0);

                }
            }
                
        }

        RCLCPP_WARN(this->get_logger(),"ePos: %f",ePos);
        RCLCPP_WARN(this->get_logger(),"eTheta: %f",eTheta);
            
    }




    void callback_alive_turtles(const my_robot_interfaces::msg::Turtles::SharedPtr msg)
    {
        alive_turtles_vector_ = msg->turtles;
    }

    my_robot_interfaces::msg::Turtle get_turtle()
    {
        my_robot_interfaces::msg::Turtle turtle;
        float min_distance = 9999.0;
        float current_distance = 9999.0;

        if ((int)alive_turtles_vector_.size() > 0)
        // if there is a turtle to catch
        {
            turtle = alive_turtles_vector_.at(0);

            if (catch_closest_turtle_first_ == true)
            {
                // search and return closest turtle
                for (int i = 1; i < (int)alive_turtles_vector_.size(); i++)
                {
                    
                    current_distance = sqrt(pow(alive_turtles_vector_.at(i).x - posX_,2)+pow(alive_turtles_vector_.at(i).y - posY_,2));
                    if (current_distance < min_distance)
                    {
                        min_distance = current_distance;
                        turtle = alive_turtles_vector_.at(i);
                    }
                }

                return turtle;
            }
            else
            {
                // return the first turtle appeared and not catched
                return alive_turtles_vector_.at(0);
            }

        }


        //if there isn't any turtle, I use try-catch block where this function is called 
        return turtle;

    }

    void callback_get_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        posX_ = msg->x;
        posY_ = msg->y;
        theta_ = msg->theta;
    }

    void publish_velocity(float linearVel, float angularVel)
    {
        geometry_msgs::msg::Twist cmd_vel_;

        cmd_vel_.linear.x = linearVel;
        cmd_vel_.angular.z = angularVel;

        publisher_cmd_vel_->publish(cmd_vel_);

    }

private:
    bool catch_closest_turtle_first_;
    bool catching_a_turtle_;

    float posX_ = 0.0;
    float posY_ = 0.0;
    float theta_ = 0.0;

    float set_point_X_ = 0.0;
    float set_point_Y_ = 0.0;

    float ePos = 0.0;
    float eTheta = 0.0;

    float difX = 0.0;
    float difY = 0.0;

    //PID parameters
    float T = 0.01;
    float P = 3;
    float P_ang = 6;
    float I = 0.1;
    float D = 0.1;

    float ePos_threshold = 0.1;
    float eTheta_threshold = 0.1;
    float eTheta_threshold2 = 10;

    my_robot_interfaces::msg::Turtle turtle_to_catch_ = my_robot_interfaces::msg::Turtle();
    rclcpp::Client<my_robot_interfaces::srv::KillTurtle>::SharedPtr service_client_catch_turtle_;
    rclcpp::Subscription<my_robot_interfaces::msg::Turtles>::SharedPtr subscriber_alive_turtles_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_turtle_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_vector_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}