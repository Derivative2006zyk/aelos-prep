#The steps of setting a FSM for your robot in ROS2_humble
## 1. creare package of FSM
```bash
    ros2 pkg create fsm --build-type ament_python --dependencies rclpy std_msgs geometry_msgs
```
## 2. write the FSM-v1
``` python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
class FSMV1Node(Node):
    def __init__(self):
        super().__init__('fsm_v1_node')
        # status list
        self.states = ['INIT', 'CALIB', 'FACE_TASK', 'AVOID_TASK', 'END']
        self.current_state_idx = 0
        # lg and time
        self.entry_times = {}  # record the first enter time
        self.max_duration = 60.0  # Max keeping time by every status⭐
        self.start_time = None  # current time begining time
        # Max restart time 2⭐
        self.retry_count = {}
        self.max_retries = 2
        # Pub /robot_state
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        # Timer: every 100ms check the status
        self.timer = self.create_timer(0.1, self.update_state)
        # Initialization Publish status
        self.log_and_publish()
    def log_and_publish(self):
        """Record timestamp and publish current status"""⭐
        now = datetime.now()
        current_state = self.get_current_state_name()
        # Record enter_time
        if current_state not in self.entry_times:
            self.entry_times[current_state] = now
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        # Print the logger with timestamp⭐
        timestamp_str = now.strftime("%H:%M:%S.%f")[:-3]
        self.get_logger().info(f"[{timestamp_str}] STATE: {current_state}")
        # Publish status
        msg = String()
        msg.data = current_state
        self.state_pub.publish(msg)
    def get_current_state_name(self):
        return self.states[self.current_state_idx]
    def update_state(self):
        current_state = self.get_current_state_name()
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        if current_state == 'INIT':
            self.current_state_idx += 1
            self.log_and_publish()
        elif current_state == 'CALIB':
            # CALIB limit keeping 3s
            if now_sec - self.start_time >= 3:
                self.current_state_idx += 1
                self.start_time = now_sec  # restart for the next time
                self.log_and_publish()
        elif current_state in ['FACE_TASK', 'AVOID_TASK']:
            # Initialization retry times
            if current_state not in self.retry_count:
                self.retry_count[current_state] = 0
            elapsed = now_sec - self.start_time
            # TIMEOUT JUDGMENT
            if elapsed >= self.max_duration:
                self.get_logger().warn(f"{current_state} TIMEOUT! (>{self.max_duration}s)")
                
                # UNOver the retry limit for restart
                if self.retry_count[current_state] < self.max_retries:
                    self.retry_count[current_state] += 1
                    self.start_time = now_sec
                    self.get_logger().info(f"Retrying {current_state} ({self.retry_count[current_state]}/2)")
                else:
                    # Over the retry limit for change stage
                    self.current_state_idx += 1
                    self.start_time = now_sec
                    self.log_and_publish()
            # Simulation task successfully completed
            # Fixed time for change stage
            elif elapsed >= 5:  # Assuming need 5 seconds to succeed
                self.current_state_idx += 1
                self.start_time = now_sec
                self.log_and_publish()
        elif current_state == 'END':
            # OVER(DOSE)
            pass
def main(args=None):
    rclpy.init(args=args)
    node = FSMV1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
```

## 3. Put up the node_enterance
``` python
    entry_points={
        'console_scripts': [
            'fsm_v1_node = fsm.fsm_v1:main',
        ],
    },
```

## 4. SUDO ADDING to link the_fsm and the_fake_robot

```bash
    chmod +x ~/ros2_ws/src/fsm/fsm/fsm_v1.py
    chmod +x ~/ros2_ws/src/robot_interface/robot_interface/fake_robot_node.py
```

## 5. Build the package:
``` bash
cd ~/ros2_ws  
colcon build --packages-select robot_interface
colcon build --packages-select fsm
source install/setup.bash  
```

## 6. Test the package:

### in the one shell(also need build)
```bash
   ros2 run robot_interface fake_robot_node
```

### in the one shell(also need build)
```bash
   ros2 run fsm fsm_v1_node
```