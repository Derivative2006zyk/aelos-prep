# The introductions of robot_interfaces with Pub_comm_skeleton by heartbeat

## 1. Create the work-space of ROS2:
- open WSL and source for setup
- And then,command 
```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
```

## 2. Create comm_skeleton_package depended by rclpy std_msgs:

``` bash
    cd ~/ros2_ws/src  
    ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs comm_skeleton_pkg
```

## 3. Write the Publisher node:

### Create file:
``` bash
    cd ~/ros2_ws/src/comm_skeleton_pkg/comm_skeleton_pkg 
    touch pub_heartbeat.py
```
### Write code:

``` python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class HeartbeatPublisher(Node):
        def __init__(self):
            super().__init__('heartbeat_publisher')
           #called/heartbeat,String,10
           self.publisher_= self.create_publisher(String,'/heartbeat',10)
          self.timer = self.create_timer(1.0,self.timer_callback)
          self.count = 0

       def timer_callback(self):
           msg = String()
          msg.data = f'Heartbeat:{self.count}'
         self.publisher_.publish(msg)
         self.get_logger().info(f'call:{msg.data}')
         self.count += 1

    def main(args=None):
       rclpy.init(args=args)
        publisher = HeartbeatPublisher()
       rclpy.spin(publisher)
      publisher.destroy_node()
       rclpy.shutdown()
    if __name__ == '__main__':
       main()
```

## 4. Put up the node_enterance
    open the file`setup.py`,add:
``` python
    entry_points={
        "console_scripts": [

            #Format:Node_name = Package_name.File_name:main

            'pub_heartbeat = comm_skeleton_pkg.pub_heartbeat:main',
        ],
    },
```

## 5. Build the package:
``` bash
    cd ~/ros2_ws  
    colcon build --packages-select comm_skeleton_pkg
    source install/setup.bash  
```

## 6. Test the package:

`ros2 run my_robot_interfaces heartbeat_pub`

# If you finished the comm_skeleton then let's write a true Pub_pkg

## 1. Create package robot_interface:

``` bash
    cd ~/ros2_ws/src  
    ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs geometry_msgs robot_interface
```

## 2. Write the Publisher node:

### Create file*3:
``` bash
    cd ~/ros2_ws/src/robot_interface/robot_interface 
    touch pub_cmd_vel.py
    touch pub_robot_state.py
    touch sub_heartbeat.py
```
### Write code_pub_cmd_vel:

``` python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    class CmdVelPublisher(Node):
        def __init__(self):
            super().__init__('cmd_vel_publisher')
            #publisher's init,mean:/cmd_vel,type:Twist,laugh:10
            self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
            self.timer = self.create_timer(2.0,self.timer_callback)
            self.twist = Twist()
     def timer_callback(self):
            self.twist.linear.x = 0.2 #linear velocity
            self.twist.angular.z = 0.5 #angular velocity
            self.publisher_.publish(self.twist)
            self.get_logger().info(f'called cmd_vel:linear.x={self.twist.linear.x},angular.z={self.twist.angular.z}')
    def main(args=None):
        rclpy.init(args=args)
        publisher = CmdVelPublisher()
      rclpy.spin(publisher)
      publisher.destroy_node()
      rclpy.shutdown()
    if __name__ == '__main__':
      main()
```
### Write code_pub_robot_state:

``` python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class RobotStatePublisher(Node):
        def __init__(self):
            super().__init__('robot_state_publisher')
            #publisher's init,mean:/robot_state,type:String,laugh:10
            self.publisher_ = self.create_publisher(String,'/robot_state_publisher',10)
            self.timer = self.create_timer(3.0,self.timer_callback)
            self.states = ['IDLE','RUNNING','ERROR','FALLEN'] #the list of robot_state
            self.state_index = 0
        def timer_callback(self):
            msg = String()
            msg.data = self.states[self.state_index]
            self.publisher_.publish(msg)
            self.get_logger().info(f'publish robot_state:{msg.data}')
            self.state_index = (self.state_index + 1) % 4 #cycle switching state
    def main(args=None):
        rclpy.init(args=args)
        publisher = RobotStatePublisher()
        rclpy.spin(publisher)
        publisher.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
```
### Write code_sub_heartbeat:

``` python
    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class HeartbeatSubscriber(Node):
        def __init__(self):
            super().__init__('heartbeat_subscriber_b')
            #subscriber's init,mean:/heartbeat,type:String,laugh:10
            self.subscription = self.create_subscription(String,'heartbeat',self.lisener_callback,10)
            self.subscription  #prevent aciidental garbage collection
        def lisener_callback(self,msg):
            self.get_logger().info(f'The student B accept the heartbeat:{msg.data}')
    def main(args=None):
        rclpy.init(args=args)
        subscriber = HeartbeatSubscriber()
        rclpy.spin(subscriber)
        subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
```
## 3. Put up the node_enterance
    same as before,write the file`setup.py`
``` python
    entry_points={
        "console_scripts": [
            'pub_cmd_vel = robot_interface.pub_cmd_vel:main', #publish/cmd_vel
            'pub_robot_state = robot_interface.pub_robot_state:main', #publish/robot_state
            'sub_heartbeat_b = robot_interface.sub_heartbeat:main',  #subscriber/heartbeat
        ],
    },
```

## 4. Build the package:
``` bash
cd ~/ros2_ws  
colcon build --packages-select robot_interface
source install/setup.bash  
```

## 5. Test the package:
Open two shell
at the shell_one running the publisher of heartbeat

``` bash
source ~/ros2_ws/install/setup.bash  
ros2 run comm_skeleton_pkg pub_heartbeat  
```

at the shell_two running the subscriber of three subject

``` bash
source ~/ros2_ws/install/setup.bash  
ros2 run robot_interface sub_heartbeat_b
```

You will see that a shell published the information of heartbeat,and the other shell subscribe the information of heartbeat from shell_one.
Congratulate!You finish your own communication interface.
Now you may doubt about the purpose of code_pub_cmd_vel and code_pub_robot_state.So I give the introduction.

 subject list:

1. **/heartbeat**  
   - type:`std_msgs/String`  
   - purpose:publish robot survival status update frequency is 1 second.  

2. **/cmd_vel**  
   - type:`geometry_msgs/Twist`  
   - purpose:control robot movement speed,`linear.x`(m/s),`angular.z`(rad/s).  

3. **/robot_state**  
   - type:`std_msgs/String`  
   - purpose:publish robot movement status,can switch`IDLE`,`RUNNING`,`ERROR`,`FALLEN`.

# Now we will achieve fake_robot through shell with a fake_robot_node
## 1. enter the work-space
``` bash
   cd ~/ros2_ws/src
```

## 2.If you don't install the package that you needed,install them first
``` bash
   ros2 pkg create --build-type ament_python robot_interfaces --dependencies rclpy geometry_msgs std_msgs
```

## 3.create the fake_node_file
``` bash
cd ~/dev_ws/src/robot_interface/robot_interface
touch fake_robot_node.py
```

## 4.Write the fake_node:
``` python
    # interfaces/ fake_robot_node.py 
    # V1:Compatible FSMv1 Controller
    # ✅ Keep the source code of V0 and only comment it out
    # ✅ New subscription /robot_state from FSM'status
    # ✅ Execute tasks based on changes in status
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    import random  # Keep to random status
    class FakeRobotNode(Node):
        def __init__(self):
            super().__init__('fake_robot_node')
            # /cmd_vel
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.cmd_vel_callback,
                10
            )
            self.get_logger().info("fake_robot_node Started,listening /cmd_vel...")
            #  /robot_state 
            self.state_pub = self.create_publisher(String, '/robot_state', 10)
            # 🔴 
            # self.create_timer(2.0, self.publish_random_state)
            # self.get_logger().info("robot_state")
            # ✅ NEW:Sub FSM publishing /robot_state
            self.robot_state_sub = self.create_subscription(
                String,
                '/robot_state',
                self.robot_state_callback,
                10
            )
            self.current_state = "UNKNOWN"
            self.get_logger().info("🟢 CONTROL by FSM /robot_state,this node only responds to changes")
        def cmd_vel_callback(self, msg: Twist):
            """
            Analyze and print motion instructions
            """
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            if linear_x > 0:
                action = f"Forward (LINEAR_velocity={linear_x:.2f} m/s)"
            elif linear_x < 0:
                action = f"Backward (LINEAR_velocity={abs(linear_x):.2f} m/s)"
            elif angular_z > 0:
                action = f"Left_turn (ANGULAR_velocity={angular_z:.2f} rad/s)"
            elif angular_z < 0:
                action = f"Right_turn (ANGULAR_velocity={abs(angular_z):.2f} rad/s)"
            else:
                action = "Stop"
            # Determine whether to take action based on the current status
            if self.current_state in ['FACE_TASK', 'AVOID_TASK', 'RUNNING', 'CALIB']:
                self.get_logger().info(f"[{self.current_state}] Perform{action}")
            else:
                self.get_logger().warn(f"[{self.current_state}] The current state ignores movement instructions{action}")
        def robot_state_callback(self, msg: String):
            """
            recieve FSM publish-status
            """
            new_state = msg.data
            old_state = self.current_state
            if new_state != old_state:
                self.current_state = new_state
                timestamp = self.get_clock().now().to_msg().sec
                self.get_logger().info(f"🔄 UPDATE state[{old_state}] → [{new_state}] (T={timestamp})")
                # Imitate initialization behavior of each state
                if new_state == 'CALIB':
                    self.simulate_calibration()
                elif new_state == 'FACE_TASK':
                    self.simulate_face_detection()
                elif new_state == 'AVOID_TASK':
                    self.enable_obstacle_avoidance()
                elif new_state == 'END':
                    self.shutdown_actions()
        def simulate_calibration(self):
            self.get_logger().info("⚙️  Virtual Calibration beginning,IMU zeroing,Servo Initialization...FINISHED")
            #虚拟校准程序，可模拟陀螺仪，电机归零等过程
        def simulate_face_detection(self):
            self.get_logger().info("📷 THE FACE Detection module is activated,BEGINS scanning the area ahead...")
            #模拟AI进行人脸检测时的延迟或成功信号
        def enable_obstacle_avoidance(self):
            self.get_logger().info("🛡️  Start Automatic obstacle avoidance simulation:radar data stream ready,path planing enabled")
            #启用虚拟避障并监听，可以模拟雷达数据生成或轨迹修正
        def shutdown_actions(self):
            self.get_logger().info("🛑 All tasks completed,execute shutdown process:Motor Shutdown,Sensor Shutdown")
            #停止，关闭设备
        # V0
        '''
        def publish_random_state(self):
            states = ["IDLE", "RUNNING", "ERROR", "FALLEN"]
            random_state = random.choice(states)
            
            msg = String()
            msg.data = random_state
            self.state_pub.publish(msg)
            self.get_logger().info(f"📢 [imitate] Self publishing status: {random_state}")
    '''
    
    def main(args=None):
        rclpy.init(args=args)
        node = FakeRobotNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    if __name__ == '__main__':
        main()

        def publish_robot_state(self):
            """Regularly publish robot_status"""
            state_msg = String()

            # Random switch status
            state_msg.data = random.choice(self.state_list)
            self.robot_state_pub.publish(state_msg)
            self.get_logger().info(f"Publish status{state_msg.data}")

    def main(args=None):
        rclpy.init(args=args)
        node = FakeRobotNode()
        rclpy.spin(node)  # Keep Node On
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == "__main__":
        main()

```

Old Version(v0):

``` python
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    import random

    class FakeRobotNode(Node):
        def __init__(self):
            super().__init__("fake_robot_node")  # Node_name

            # 01. /cmd_vel
            self.cmd_vel_sub = self.create_subscription(
                Twist, "/cmd_vel", self.cmd_vel_callback, 10  # QoS=10
            )

            # 02. /robot_state
            self.robot_state_pub = self.create_publisher(String, "/robot_state", 10)

            # 03. Timer : every 5s publish state

            #当前由状态机控制,If you want unlock the fsm :self.timer = self.create_timer(5.0, self.publish_robot_state)

            # state_list

            self.state_list = ["IDLE", "RUNNING", "ERROR", "FALLEN"]
            self.get_logger().info("Fake Robot Node is ready!")

        def cmd_vel_callback(self, msg: Twist):
            """Sub/cmd_vel 's backcall:use to control and print state"""
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            # including speed judge command
            if linear_x > 0:
                self.get_logger().info(f"Recieve Forward_command,linear_speed={linear_x}m/s")
            elif linear_x < 0:
                self.get_logger().info(f"Recieve Retreat_command,linear_speed={linear_x}m/s")
            if angular_z > 0:
                self.get_logger().info(f"Recieve Left_command,linear_speed={angular_z}rad/s")
            elif angular_z < 0:
                self.get_logger().info(f"Recieve Right_command,linear_speed={angular_z}rad/s")

            # Stop_command
            if linear_x == 0 and angular_z == 0:
                self.get_logger().info("Recieve Stop_command")
```
## 5.Put up the node_enterance

open the file`setup.py`,and
``` python
    entry_points={
        'console_scripts': [
            'fake_robot_node = robot_interface.fake_robot_node:main',
        ],
    },
```

## 6.Build the package:
``` bash
    cd ~/ros2_ws
    colcon build --packages-select robot_interface
    source install/setup.bash
```
## 7.Test the package:
### in one shell
``` bash
   ros2 run robot_interface fake_robot_node
```
### in the other shell,also need enter the source
#### 1.pub forward command
``` bash
  ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
#### 2.pub stop command
``` bash
  ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
#### 3.view the status
``` bash
  ros2 topic echo /robot_state
```

# Then try to complete some test for fake_robot_node
## 1. enter content of package and create `test` file
``` bash
   cd ~/ros2_ws/src/robot_interface/robot_interface
   mkdir test
   cd test
   touch test_fake_robot.py
```

## 2. Write the test_code:
``` python
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    import pytest
    from robot_interfaces.fake_robot_node import FakeRobotNode

    class TestNode(Node):
        def __init__(self):
            super().__init__("test_node")
            self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.robot_state_sub = self.create_subscription(
                String, "/robot_state", self.state_callback, 10
            )
            self.received_state = None
        def state_callback(self, msg):
            self.received_state = msg.data
        def publish_cmd_vel(self, linear_x, angular_z):
            """pub cmd_vel information"""
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            self.cmd_vel_pub.publish(twist_msg)
    # initialize ROS2 source (before all of tests)
    @pytest.fixture(scope="module")
    def ros_context():
        rclpy.init()
        yield
        rclpy.shutdown()
    # test 1: recieve STOP command(linear + augular = 0), node print "stop" log
    def test_stop_command(ros_context, capsys):

        # ON Fake Robot_node and teat_node
        fake_node = FakeRobotNode()
        test_node = TestNode()
        executor = SingleThreadedExecutor()
        executor.add_node(fake_node)
        executor.add_node(test_node)
        # publish STOP command
        test_node.publish_cmd_vel(linear_x=0.0, angular_z=0.0)
        # trigger callback(simulate ROS2 roop)
        executor.spin_once(timeout_sec=1.0)
        # check node_print
        captured = capsys.readouterr()
        assert "recieve stop command" in captured.out
        # break the node
        fake_node.destroy_node()
        test_node.destroy_node()
    # test 2: recieve FORWARD command, node print "forward" log

    def test_forward_command(ros_context, capsys):
        fake_node = FakeRobotNode()
        test_node = TestNode()
        executor = SingleThreadedExecutor()
        executor.add_node(fake_node)
        executor.add_node(test_node)

        test_node.publish_cmd_vel(linear_x=0.5, angular_z=0.0)
        executor.spin_once(timeout_sec=1.0)

        captured = capsys.readouterr()
        assert "recieve forward_command,linear =0.5m/s" in captured.out

        fake_node.destroy_node()
        test_node.destroy_node()

    # test 3: recieve LEFT command, node print "left" log 

    def test_turn_left_command(ros_context, capsys):
        fake_node = FakeRobotNode()
        test_node = TestNode()
        executor = SingleThreadedExecutor()
        executor.add_node(fake_node)
        executor.add_node(test_node)

        test_node.publish_cmd_vel(linear_x=0.0, angular_z=0.3)
        executor.spin_once(timeout_sec=1.0)

        captured = capsys.readouterr()
        assert "recieve left_command,augular=0.3rad/s" in captured.out

        fake_node.destroy_node()
        test_node.destroy_node()

    # test 4:publish status in regular time,and check if the status is on the list

    def test_state_publish(ros_context):
        fake_node = FakeRobotNode()
        test_node = TestNode()
        executor = SingleThreadedExecutor()
        executor.add_node(fake_node)
        executor.add_node(test_node)
        # wait the timer turn on(make 5second and 1second to buffer)
        executor.spin_once(timeout_sec=6.0)
        # check if the status that was recieved is on the allowed list

        allowed_states = ["IDLE", "RUNNING", "ERROR", "FALLEN"]
        assert test_node.received_state in allowed_states

        fake_node.destroy_node()
        test_node.destroy_node()
```

## 3.Put up the node_enterance
### open the file`setup.py`,and
``` python
   setup(

       # ... others ...
       pytest_requirements=["pytest"],
       tests_require=["pytest"],
       entry_points={
           # ... console_scripts ...

       },
   )
```

### open the file`package.xml`,and add:
``` xml
   <test_depend>ament_pytest</test_depend>
   <test_depend>pytest</test_depend>
```

### back to `ros2_ws`
``` bash
   cd ~/ros2_ws
   colcon build --packages-select robot_interface
```

## 4.Test by the test_code:
``` bash
   colcon test --packages-select robot_interface
```

And then you can see your result including all previous
``` bash
    colcon test-result
``` 