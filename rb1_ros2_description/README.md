# The rb1_ros2_description package

This package contains a ROS2 robot model description for Robotnik's RB-1 mobile base.   

## Disclaimer:  
This package only modifies/adapts files from these repositories/packages:  
- [RobotnikAutomation/rb1_base_sim](https://github.com/RobotnikAutomation/rb1_base_sim) licensed under the BSD 2-Clause "Simplified" License
- [RobotnikAutomation/rb1_base_common/rb1_base_description](https://github.com/RobotnikAutomation/rb1_base_common/tree/melodic-devel/rb1_base_description), licensed under the BSD License
- [RobotnikAutomation/robotnik_sensors],(https://github.com/RobotnikAutomation/robotnik_sensors) licensed under the BSD License




# Overview of the rb1_ros2_description package:
    This package is designed to activate the RB1 Gazebo simulation & activate 3 controllers to use with the RB1 model:
        - Joint State Broadcaster (For reading the joints state throughout the lifecycle of the simulation)
        - diff_drive_controller (Differential Drive Controller allowing for Twist commands to be sent RB1 enabling linear/angluar movement)
        - rb1_lift_controller (Custom Controller designed to lift & lower elevator joint)
    The package also contains a config file for easy editing of the controllers parameters as well as a launch file that will be used to
    activate the controllers





# How to start the simulation & activate the controllers:
    The launch file found inside the package named "rb1_ros2_xacro.launch.py" is used to start the simulation as well as activate the 
    controller.

    Nodes responsible for starting Simulation:
        - "gz_sim" --> launches gazebo and spawns empty world utilizing the "ros_gz_sim" package
        - "rsp_robot" --> robot state publisher node that accepts the various RB1 URDF files to generate model content on /robot_description
                            which is used to generate the actual model of the RB1 inside Gazebo
        - "gz_spawn_entity" --> also uses the "ros_gz_sim" package to generate the RB1 model in gazebo based off the URDF found on the 
                                    /robot_description topic

    Nodes responsible for activating Controllers:
        The following nodes all utilize the "RegisterEventHandler" module from launch.actions. Event handlers are registered for each controller
        in order to activate them after the previous controller activation ends regardless if it succeeded or not. This allows the system to activate each
        controller one at a time and prevents one controller activation from overlapping another and crashing the system.
        
        - joint_state_broadcaster_spawn --> activates the joint state broadcaster used to read current joint states
        - diff_drive_spawner --> activates the diff_drive_controller (found inside the ros2_controllers repo). Used for diff drive commands
        - rb1_lift_controller_spawner --> activates the custom lift controller for the rb1 robot

        All controllers stem from the "controller_manager" package since the controller manager is the orchestrator of all controllers registered
        within the ros2_control framework

    In order to start the simulation & activate the controllers the you need to run 3 commands inside the first terminal:
        1.) cd ~/ros2_ws
        2.) colcon build && source install/setup.bash
    If compilation succeed execute the launch file:
        3.) ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py

    After a few moments, the Gazebo simulation should start up & spawn the RB1 model entity. Also logs regarding the success of activating
    the controllers should appear colorized within the terminal as well. If all goes well, 3 green controller activation messages should be in the logs





# How to move the robot around
    The activated Differential Drive Controller (diff_drive_controller) is reponsible for handling commands sent to the left/right drive 
    wheel joints.

    After the controller is activated, commands can be sent to the "/diff_drive_controller/cmd_vel" topic to move RB1.
    
    Example message to move RB1 by an angular velocity of 0.2 m/s:
        ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "header:
        stamp:
            sec: 0
            nanosec: 0
        frame_id: ''
        twist:
        linear:
            x: 0.0
            y: 0.0
            z: 0.0
        angular:
            x: 0.0
            y: 0.0
            z: 0.2
        "
    Similar command can be used to stop RB1 by setting both linear & angular velocities to 0.0 m/s
        ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "header:
        stamp:
            sec: 0
            nanosec: 0
        frame_id: ''
        twist:
        linear:
            x: 0.0
            y: 0.0
            z: 0.0
        angular:
            x: 0.0
            y: 0.0
            z: 0.0
        "




# How to move the lift joint
    The custom lift controller (rb1_lift_controller) is strickly used to elevate & lower the lift joint on RB1. The controller is set up 
    to be able to be used in 2 different modes: POSITION & VELOCITY.
    In this iteration of the controller, the mode is set inside the header file rb1_lift_controller.hpp with the line:
        Mode mode_{Mode::POSITION};
    Which in this case defaults the controller mode to POSITION. Changing the header code to "mode::VELOCITY" will switch the mode.
    This design should not make it to production & is only applicable in this development version of the controller. In future versions, 
    mode should be extracted to a controller parameter that can be set via the config file or a user interface should be added that allows 
    for dynamically switching modes during runtime.

    In the default POSITION mode, we need to send a displacement value to the controller via the "/rb1_lift_controller/lift_command" topic.
    Important Note: we are sending a displacement value as the command to the controller, meaning we are sending how many meters we want the
    lift to move relative to it's current position. The displacements value DOES NOT represent an absolute positional value. 

    For example, if we send a displacement value of 0.005m to the controller, we are telling it to elevate the lift 5 millimeters vertically
    from it's current position. We are not telling the lift to go to 5 millimeters away from the base of the joint. Same goes for lowering
    the lift, if we send a displacement value of -0.005m we are telling the controller to lower the lift 5 millimeters down from it's
    current position. 

    The upper limit of the lift joint is 0.034m & the lower limit is 0.0m. Therefore, if the displacement commands sent to the lift joint 
    accrue to try to lift the joint higher than 0.034m or lower than 0.0m, the lift will clamp down on it's limits and not move and further. 

    An example message to send to the lift_command topic:
        ros2 topic pub -r 10 /rb1_lift_controller/lift_command control_msgs/msg/JointJog "{joint_names: ['robot_elevator_platform_joint'], displacements: [0.034]}" 
    This command displaces the lift by 0.034m meaning that it will lift the joint all the way to its upper limit despite current position of
    the lift

    Lift can be lowered all the way back down via the same command but instead sending a negative valued displacement:
        ros2 topic pub -r 10 /rb1_lift_controller/lift_command control_msgs/msg/JointJog "{joint_names: ['robot_elevator_platform_joint'], displacements: [-0.034]}"


    ## The command message type sent to "lift_command" is of type JointJog. This means the message possess a displacements array (used
    in the default POSITION mode) as well as a velocities array (used to send velocities to the designated joints)

    If controller is switched to VELOCITY mode, the message needs to use the velocities array to send a m/s value like so:
        ros2 topic pub -r 10 /rb1_lift_controller/lift_command control_msgs/msg/JointJog "{joint_names: ['robot_elevator_platform_joint'], velocities: [0.02]}"
    The previous message will send a velocity command to the lift joint to move vertically by 0.02 m/s.
    Can again switch the sign of the velocity value to send the lift back down:
        ros2 topic pub -r 10 /rb1_lift_controller/lift_command control_msgs/msg/JointJog "{joint_names: ['robot_elevator_platform_joint'], velocities: [-0.02]}"