from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    package_ = "turtlesim_catch_all_turtles"

    turtle_Spawner_node = Node(
        package=package_,
        executable="turtleSpawner",
        name="turtleSpawner",
        parameters=[
            {"spawn_frequency":1},
            {"turtle_name_prefix":"Turtwig"},
            {"use_sim_time":False}
        ]
    )

    turtle_Controller_node = Node(
        package=package_,
        executable="turtleController",
        name="turtleController",
        parameters=[
            {"catch_closest_turtle_first":True},
            {"use_sim_time":False}
        ]
    )

    turtleSim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
    )

    ld.add_action(turtleSim_node)
    ld.add_action(turtle_Spawner_node)
    ld.add_action(turtle_Controller_node)

    return ld

