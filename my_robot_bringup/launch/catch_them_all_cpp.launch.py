from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    package_ = "turtlesim_catch_all_turtle_cpp"

    turtle_Spawner_node = Node(
        package=package_,
        executable="turtle_spawner",
        name="turtle_spawner",
        parameters=[
            {"spawn_frequency": 0.5},
            {"turtle_name_prefix":"Torterra"},
            {"use_sim_time":False}
        ]
    )

    turtle_Controller_node = Node(
        package=package_,
        executable="turtle_controller",
        name="turtle_controller",
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

