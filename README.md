# ROS2_Challenge1
Turtlesim simulation with ROS2: Catching turtles

A simulation where a turtle moves and catches other turtles. Coded in python and c++.

Nodes:
- Turtle spawner: Creates and deletes turtles.
- Turtle controller: Controls movement of main turtle. Asks Turtle spawner for "killing" a turtle catched

Roslaunch:
- Name: catch_them_all.launch.py
  - Nodes to launch with their parameters:
    - turtleSpawner:
      - "spawn_frequency"
      - "turtle_name_prefix"
      - "use_sim_time"
    - turtleController
      - "catch_closest_turtle_first"
      - "use_sim_time"
    - turtlesim

- Name: catch_them_all_cpp.launch.py
  Idem catch_them_all.launch.py but using cpp files
  
Msgs and Srvs:
- Turtle.msg
- Turtles.msg
- KillTurtle.srv
