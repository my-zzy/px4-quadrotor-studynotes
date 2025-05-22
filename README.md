# px4-quadrotor-studynotes
px4 gazebo ros2 ubuntu22.04

## Overview

This note records how to launch and control a quadrotor with PX4. 

Fork from https://github.com/Jaeyoung-Lim/px4-offboard. 

## Prerequisites

   * Ubuntu 22.04
   * PX4-Autopilot (I cloned main branch in 2025.05) https://docs.px4.io/main/en/ros2/user_guide.html
   * ROS2 Humble (gazebo sim 8 automatically installed with ros2)**(put source /opt/ros/humble/setup.bash into ~/.bashrc)**
   * Micro XRCE-DDS Agent & Client
   * QGroundControl https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
   * Python 3.10(or other suitable version)


## Install PX4 Offboard and dependencies


## Create workspace

```
mkdir -p ~/px4_ros_com_ws/src && cd ~/px4_ros_com_ws/src
```

### Download px4-offboard (this repo)

```
cd ~
git clone https://github.com/my-zzy/px4-quadrotor-studynotes.git
```

### Install PX4 msg

The `px4-offboard` example requires `px4_msgs` definitions. Remember to source it before you launch this repo. 

```
git clone https://github.com/PX4/px4_msgs.git
```

Build:

```
colcon build
```

This should build. You may see some warnings interspered with the output.  As long as there are no __*errors*__ you should be OK..


## Run the Demo

### Overview

You need 4 terminal windows:

   * Micro XRCE-DDS
   * px4 make gazebo
   * px4-offboard example
   * QGroundControl

In each terminal window, we will first source any workspace settings required for a particular component and also set the `ROS_DOMAIN_ID` and `PYTHONOPTIMIZE`  in each window (TODO: not yet understand what these two parameters mean). 

### Start the Micro XRCE-DDS

In the terminal you designated as the `Micro XRCE-DDS` terminal:

```
cd ~/path/to/dds
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
MicroXRCEAgent udp4 -p 8888 ROS_DOMAIN_ID=0
```
Or
```
MicroXRCEAgent udp4 -p 8888
```

### Start Gazebo

In the terminal you designated for gazebo:

```
cd ~/PX4-Autopilot
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
make px4_sitl gz_x500
```
Or
```
make px4_sitl gz_x500
```

Gazebo should start and you will see a big PX4 ascii art banner in the gazebo terminal and the GUI will launch.

Back in the  `Micro XRCE-DDS`-ros-agent` terminal, you should see the Micro ROS Agent start to receive DDS messages:


Leave the agent running. 

If you scroll up in the Gazebo terminal window, you should see logs indicating it set up the microdds_client:

```
INFO  [microdds_client] synchronized with time offset 1680366210257897us
INFO  [microdds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 79
...
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
...
```

### Start QGround Controller and Take Off

In the terminal you designated as the `QGroundControl` terminal, launch the app:

```
cd /dir/where/qgroundcontroller/is/installed
./QGroundControl.AppImage
```

Click `Ready` on the left-top corner, and click `Arm`. 

Click `HOLD` to change mode to `Offboard`. If you can't find Offboard, go into setting and open it.


### Start the px4-offboard example

#### Check ROS Messages

Before we start the example, lets check ROS2 topics.

In the window you have designated for the `px4-offboard` example:


```
cd ~/px4_ros_com_ws
source install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONOPTIMIZE=1
ros2 topic list
```

You should see a list of topics that match the ones sent from Gazebo.:

```
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
...
/fmu/out/failsafe_flags
/fmu/out/sensor_combined
...
/parameter_events
/rosout
```

If you do not see the topics:

  * Check that `ROS_DOMAIN_ID=0`  in all the terminals

Let's echo a topic:

```
ros2 topic echo /fmu/out/vehicle_odometry
```

The terminal should echo some rapidly updating details about the simulated drone.  If you look at position, you can see it matches the height of our drone:

```
timestamp: 1680367557988752
timestamp_sample: 1680367557988752
pose_frame: 1
position:
- 0.023736324161291122
- -0.007955201901495457
- -9.922133445739746
q:
- 0.6887969374656677
- 0.002538114320486784
- -0.007746106944978237
- 0.7249085307121277
...
```

*In this case the `-9.922133445739746` value indicates the drone in the gazebo sim is in the air.*

Now that we verfied the DDS-ROS subscription communication link, we can start the demo

CTRL-C to stop the topic echo and then:

<!-- ```
source ../px4_ros_com_ws/install/setup.bash
source install/setup.bash
```

The first source reensures the dependencies are loaded for the demo.
The second is for the demo itself. -->

#### Launch the Demo

```
source install/setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```

If things work, the demo should immediately launch an RViz window with the 3D axis indicator (red, green blue color) at the top of the window above the grid.  This indicates the drone's position in the gazebo sim/

The terminal should show:

```
[INFO] [launch]: All log files can be found below /home/analyst/.ros/log/2023-04-01-16-50-55-835808-casa-32775
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [visualizer-1]: process started with pid [32776]
[INFO] [offboard_control-2]: process started with pid [32778]
[INFO] [rviz2-3]: process started with pid [32780]
[rviz2-3] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[offboard_control-2] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT is deprecated. Use ReliabilityPolicy.BEST_EFFORT instead.
[offboard_control-2]   warnings.warn(
[offboard_control-2] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL is deprecated. Use DurabilityPolicy.TRANSIENT_LOCAL instead.
[offboard_control-2]   warnings.warn(
[offboard_control-2] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST is deprecated. Use HistoryPolicy.KEEP_LAST instead.
[offboard_control-2]   warnings.warn(
[rviz2-3] [INFO] [1680367856.767339081] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-3] [INFO] [1680367856.767961910] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[visualizer-1] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT is deprecated. Use ReliabilityPolicy.BEST_EFFORT instead.
[visualizer-1]   warnings.warn(
[visualizer-1] /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/qos.py:307: UserWarning: HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST is deprecated. Use HistoryPolicy.KEEP_LAST instead.
[visualizer-1]   warnings.warn(
[rviz2-3] [INFO] [1680367856.847242023] [rviz2]: Stereo is NOT SUPPORTED
```

Now head back to QGroundControl and enable offboard control.  Click the current mode "HOLD" in upper left, then in the menu, select "Offboard":

After a 1-2 sec pause, the demo should take control and you should see the 3d indicator in Rviz drawing circles.





## Guide and References

### Paper reference

Adaptive Backstepping-Based Trajectory Tracking Control for Quadrotor UAV with Uncertainty Disturbance 

-- Zhiming Chen, Longwu Liu, and Zhouhuai Luo

### Doc Reference

   * Publish & choose offboard control modes https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages

   * trajectorysetpoint message https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint.html

   * Low-level control of each hover's thrust message https://docs.px4.io/main/en/msg_docs/ActuatorMotors.html

   * Low-level control of thrust and torque: VehicleThrustSetpoint & VehicleTorqueSetpoint https://docs.px4.io/main/en/msg_docs/VehicleThrustSetpoint.html#vehiclethrustsetpoint-uorb-message

   * Details about the drone https://github.com/PX4/PX4-gazebo-models/tree/main/models (Note: Px4 uses relative or normalized value. Mapping if needed.)

   * Frame conventions https://docs.px4.io/main/en/ros2/user_guide.html#ros-2-px4-frame-conventions

```
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
```

```
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.02166666666666667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.02166666666666667</iyy>
          <iyz>0</iyz>
          <izz>0.04000000000000001</izz>
        </inertia>
      </inertial>
```

```
      <visual name="5010_motor_base_0">
        <pose>0.174 0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
```

### How to add a Node

1. Create a file new_file.py in px4_offboard folder

2. Add a Node in offboard_position_control.launch.py

```
Node(
   package='px4_offboard',
   namespace='px4_offboard',
   executable='new_file',  # <-- NEW script name
   name='control',
),
```

3. Edit setup.py
```
entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'visualizer = px4_offboard.visualizer:main',
                'new_file = px4_offboard.new_file:main',  # <-- this is new
        ],
    },
```

4. Build the workspace again
