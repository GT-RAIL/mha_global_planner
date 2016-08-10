# mha_global_planner
A nav_core global planner implementing SMHA*, using SBPL. [SBPL](http://sbpl.net/) is a library of planners, and this package wraps their implementation of Shared Multi-Heuristic A* as a ros global planner.
To build and run this, you will need the sbpl as well. I recommend using the sbpl_catkin package, which can be found [here](https://github.com/aurone/sbpl_catkin) on github.

# Dependencies for running

The out-of-the-box demo uses the Stage simulator and the turtlebot, so install those first

    sudo apt-get install ros-indigo-turtlebot-stage
    
You also probably want the [nav_points](https://github.com/PeterMitrano/nav_points) package. Clone it into your catkin_ws

# Building

    # from catkin workspace
    cd src
    git clone https://github.com/GT-RAIL/mha_global_planner.git
    git clone https://github.com/aurone/sbpl_catkin.git
    cd sbpl_catkin
    git submodule init
    git submodule update
    
    # go back to catkin workspace
    catkin_make mha_global_planner
    
# Launch files

The out-of-the-box setup can be run as follow:

1) start stage and rviz and lots of other goodness

    roslaunch mha_global_planner turtlebot_in_stage.launch
    
2) start move_base and the planners. It will start with an additional `avoid_square_heuristic`.

    roslaunch mha_global_planner move_base.launch
    
3) Click on some of the markers and watch the robot drive around. To visualize the square the robot is avoiding, you can run

    rostopic pub /polygon geometry_msgs/PolygonStamped -f param/polygon.yaml -l



