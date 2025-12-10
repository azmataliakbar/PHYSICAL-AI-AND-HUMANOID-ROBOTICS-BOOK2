// frontend/src/data/bookContent.ts

export interface Chapter {
  id: number;
  title: string;
  part: 1 | 2 | 3;
  partName: string;
  startPage: number;
  endPage: number;
  content: string;
  difficulty: 'Easy' | 'Medium' | 'Hard';
  estimatedTime: string;
  objectives: string[];
  keywords: string[];
}

export const bookData: Chapter[] = [
  // ==========================================
  // PART 1: FOR STUDENTS (10 Chapters, Pages 5-31)
  // ==========================================
  
  {
    id: 1,
    title: "What is Physical AI?",
    part: 1,
    partName: "For Students",
    startPage: 5,
    endPage: 6,
    difficulty: "Easy",
    estimatedTime: "10 minutes",
    objectives: [
      "Understand what Physical AI means",
      "Learn the difference between digital and physical AI",
      "Explore real-world applications"
    ],
    keywords: ["Physical AI", "Embodied AI", "Robotics", "Humanoids"],
    content: `
# Chapter 1: What is Physical AI?

## 🤖 Welcome to the Future!

**Physical AI** refers to artificial intelligence systems that exist in and interact with the physical world through robotic bodies. Unlike digital AI that lives purely in computers, Physical AI has a body—it can move, sense, and manipulate objects.

### What Makes It "Physical"?

💡 **Key Difference:**
- **Digital AI**: ChatGPT, image generators, recommendation systems
- **Physical AI**: Humanoid robots, robotic arms, autonomous vehicles

### Real-World Examples

🏭 **Manufacturing:**
- Assembly line robots
- Quality inspection systems
- Automated warehouses

🏥 **Healthcare:**
- Surgical robots (Da Vinci system)
- Rehabilitation assistants
- Elderly care robots

🏠 **Home:**
- Vacuum cleaning robots (Roomba)
- Lawn mowing robots
- Companion robots

🚗 **Transportation:**
- Self-driving cars
- Delivery drones
- Autonomous ships

### Why Humanoid Robots?

Humanoid robots (robots that look and move like humans) are special because:

1. **Human Environment Design**: Our world is built for humans—stairs, doorknobs, chairs. Humanoid robots can navigate these spaces naturally.

2. **Intuitive Interaction**: People feel more comfortable interacting with human-like robots.

3. **Versatility**: One humanoid can perform multiple tasks that would require several specialized robots.

4. **Research Platform**: Studying humanoids helps us understand human movement and intelligence.

### The Challenge

Creating Physical AI is MUCH harder than digital AI because:

⚠️ **Real-World Constraints:**
- Gravity and physics
- Limited battery power
- Mechanical wear and tear
- Safety requirements
- Unpredictable environments

✨ **But the Rewards:**
- Robots that can help in disasters
- Assistants for disabled individuals
- Exploration of dangerous environments
- Automation of physical labor

### Your Journey Starts Here

In this book, you'll learn to build and program Physical AI systems from scratch. Let's begin! 🚀

---

## 📚 Summary

- Physical AI = AI with a body that interacts with the physical world
- Humanoid robots are designed to work in human environments
- Applications range from manufacturing to healthcare
- Building Physical AI requires understanding both software and hardware

**Next:** Chapter 2 - Introduction to ROS2
`
  },

  {
    id: 2,
    title: "Introduction to ROS2",
    part: 1,
    partName: "For Students",
    startPage: 7,
    endPage: 9,
    difficulty: "Easy",
    estimatedTime: "20 minutes",
    objectives: [
      "Understand what ROS2 is and why it exists",
      "Learn basic ROS2 concepts: nodes, topics, messages",
      "Write your first ROS2 program"
    ],
    keywords: ["ROS2", "Nodes", "Topics", "Publishers", "Subscribers", "Middleware"],
    content: `
# Chapter 2: Introduction to ROS2

## 🐢 What is ROS2?

**ROS2** (Robot Operating System 2) is NOT an operating system! It's a **middleware framework** that helps robot software components talk to each other.

### Why Do We Need ROS2?

Imagine building a humanoid robot:
- Camera node reads images
- Vision node detects objects
- Planning node decides actions
- Control node moves motors

💡 **Without ROS2:** You'd need to write custom code for each connection.

✅ **With ROS2:** Everything connects automatically through a standard system!

---

## 🏗️ Core Concepts

### 1. Nodes

A **node** is a single process that performs a specific task.

**Example Nodes:**
- \`camera_node\` - Captures images
- \`object_detector\` - Finds objects in images
- \`motor_controller\` - Controls robot joints

📝 **Think of nodes as LEGO blocks** - each does one thing well, and you combine them to build complex systems.

### 2. Topics

**Topics** are named channels for sending messages between nodes.

\`\`\`
[Camera Node] ---> /camera/image ---> [Vision Node]
                    (Topic)
\`\`\`

- Topics use **publish/subscribe** pattern
- Multiple nodes can publish or subscribe to the same topic
- Messages are sent without knowing who receives them

### 3. Messages

**Messages** are the data structures sent over topics.

Common ROS2 message types:
- \`std_msgs/String\` - Text data
- \`sensor_msgs/Image\` - Camera images
- \`geometry_msgs/Twist\` - Movement commands

---

## 💻 Your First ROS2 Program

### Installation Check

First, verify ROS2 is installed:

\`\`\`bash
ros2 --version
\`\`\`

Expected output: \`ros2 cli version: ...\`

### Publisher Example (Python)

Create a file: \`talker.py\`

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        
        # Create publisher
        self.publisher = self.create_publisher(String, 'chatter', 10)
        
        # Create timer (publish every 1 second)
        self.timer = self.create_timer(1.0, self.publish_message)
        
        self.counter = 0
    
    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS2! Count: {self.counter}'
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        self.counter += 1

def main():
    rclpy.init()
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

### Subscriber Example (Python)

Create a file: \`listener.py\`

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.message_callback,
            10
        )
    
    def message_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

### Running Your First System

**Terminal 1:**
\`\`\`bash
python3 talker.py
\`\`\`

**Terminal 2:**
\`\`\`bash
python3 listener.py
\`\`\`

🎉 **You should see:**
- Talker publishing messages
- Listener receiving and printing them!

---

## 🛠️ Useful ROS2 Commands

\`\`\`bash
# List all running nodes
ros2 node list

# List all active topics
ros2 topic list

# See messages on a topic
ros2 topic echo /chatter

# Get info about a topic
ros2 topic info /chatter

# Publish to a topic manually
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello!'"
\`\`\`

---

## 📚 Summary

- **ROS2** = Middleware for robot software communication
- **Nodes** = Individual programs/processes
- **Topics** = Named communication channels
- **Messages** = Data structures sent between nodes
- **Publish/Subscribe** = Decoupled communication pattern

**Next:** Chapter 3 - Linux Basics for Robotics

---

## ✍️ Exercise

**Task:** Modify the talker to also publish the current time along with the count.

**Hint:** Use Python's \`datetime\` module!
`
  },

  {
    id: 3,
    title: "Linux Basics for Robotics",
    part: 1,
    partName: "For Students",
    startPage: 10,
    endPage: 12,
    difficulty: "Easy",
    estimatedTime: "15 minutes",
    objectives: [
      "Master essential Linux terminal commands",
      "Understand file system navigation",
      "Learn package management basics"
    ],
    keywords: ["Linux", "Terminal", "Bash", "Ubuntu", "Commands"],
    content: `
# Chapter 3: Linux Basics for Robotics

## 🐧 Why Linux?

**99% of robots run on Linux!** Here's why:

✅ Open source and free
✅ Excellent hardware support
✅ ROS2 works best on Linux
✅ Powerful command-line tools
✅ Active robotics community

**Most Common:** Ubuntu 22.04 LTS (Jammy Jellyfish)

---

## 🖥️ Essential Terminal Commands

### Navigation

\`\`\`bash
# Print current directory
pwd

# List files
ls
ls -la          # Detailed list with hidden files
ls -lh          # Human-readable file sizes

# Change directory
cd /home/user/robot_ws
cd ..           # Go up one level
cd ~            # Go to home directory
cd -            # Go to previous directory

# Create directory
mkdir my_robot
mkdir -p robot_ws/src    # Create nested directories
\`\`\`

### File Operations

\`\`\`bash
# Create empty file
touch robot.py

# Copy files
cp source.py destination.py
cp -r folder1/ folder2/    # Copy directory recursively

# Move/Rename
mv old_name.py new_name.py
mv file.py /home/user/     # Move to another location

# Delete files
rm file.py
rm -r folder/              # Delete directory
rm -rf folder/             # Force delete (be careful!)

# View file contents
cat file.txt               # Print entire file
less file.txt              # View with scrolling (q to quit)
head file.txt              # First 10 lines
tail file.txt              # Last 10 lines
tail -f log.txt            # Follow file updates (great for logs!)
\`\`\`

### Search and Find

\`\`\`bash
# Find files by name
find . -name "*.py"
find /home/user -name "robot*"

# Search inside files
grep "robot" file.txt
grep -r "ROS2" .           # Recursive search in current dir

# Which command
which python3              # Find location of executable
\`\`\`

---

## 📦 Package Management

### APT (Advanced Package Tool)

\`\`\`bash
# Update package list
sudo apt update

# Upgrade installed packages
sudo apt upgrade

# Install package
sudo apt install ros-humble-desktop

# Remove package
sudo apt remove package-name

# Search for packages
apt search robot

# Show package info
apt show ros-humble-desktop
\`\`\`

### PIP (Python Package Manager)

\`\`\`bash
# Install Python package
pip3 install numpy

# Install specific version
pip3 install numpy==1.24.0

# List installed packages
pip3 list

# Uninstall
pip3 uninstall numpy
\`\`\`

---

## 🔧 Process Management

\`\`\`bash
# List running processes
ps aux
ps aux | grep python       # Find Python processes

# Kill process
kill PID                   # Replace PID with process ID
kill -9 PID                # Force kill

# Check system resources
top                        # Interactive process viewer
htop                       # Better top (install: sudo apt install htop)

# Background processes
python3 script.py &        # Run in background
jobs                       # List background jobs
fg                         # Bring to foreground
\`\`\`

---

## 🌐 Networking Commands

\`\`\`bash
# Check network interfaces
ifconfig
ip addr

# Test connectivity
ping google.com
ping 192.168.1.100

# Check open ports
netstat -tuln
ss -tuln

# Download files
wget https://example.com/file.zip
curl -O https://example.com/file.zip
\`\`\`

---

## ⚙️ Environment Variables

\`\`\`bash
# View environment variable
echo $HOME
echo $PATH

# Set temporary variable
export MY_ROBOT="Humanoid"

# Add to PATH
export PATH=$PATH:/home/user/my_scripts

# Make permanent (add to ~/.bashrc)
nano ~/.bashrc
# Add: export MY_VAR="value"
source ~/.bashrc           # Reload config
\`\`\`

---

## 📝 Text Editors

### Nano (Beginner-Friendly)

\`\`\`bash
nano file.py
# Ctrl+O to save
# Ctrl+X to exit
\`\`\`

### Vim (Power User)

\`\`\`bash
vim file.py
# Press 'i' to insert mode
# Press Esc, then ':wq' to save and quit
# Press Esc, then ':q!' to quit without saving
\`\`\`

### VS Code (Recommended for Beginners)

\`\`\`bash
code .                     # Open current directory
\`\`\`

---

## 🔐 Permissions

\`\`\`bash
# View permissions
ls -l file.py
# Output: -rwxr-xr-x
# r=read, w=write, x=execute

# Change permissions
chmod +x script.sh         # Make executable
chmod 755 file.py          # rwxr-xr-x
chmod 644 file.txt         # rw-r--r--

# Change owner
sudo chown user:group file.py
\`\`\`

---

## 💡 Pro Tips

**1. Tab Completion**
- Start typing a command/path and press Tab
- Saves time and prevents typos!

**2. Command History**
\`\`\`bash
history                    # Show command history
!123                       # Run command #123 from history
Ctrl+R                     # Search command history
\`\`\`

**3. Pipes and Redirection**
\`\`\`bash
ls -la | grep robot        # Pipe output to grep
echo "Hello" > file.txt    # Overwrite file
echo "World" >> file.txt   # Append to file
\`\`\`

**4. Aliases (add to ~/.bashrc)**
\`\`\`bash
alias ll='ls -lah'
alias gs='git status'
alias robot='cd ~/robot_ws && source install/setup.bash'
\`\`\`

---

## 📚 Summary

✅ Mastered navigation: \`cd\`, \`ls\`, \`pwd\`
✅ File operations: \`cp\`, \`mv\`, \`rm\`
✅ Package management: \`apt\`, \`pip3\`
✅ Process control: \`ps\`, \`kill\`, \`top\`
✅ Environment variables and permissions

**Next:** Chapter 4 - Understanding URDF

---

## ✍️ Exercise

**Practice these commands:**

1. Create a directory structure: \`robot_ws/src/my_robot\`
2. Create a Python file: \`touch robot_ws/src/my_robot/main.py\`
3. Make it executable: \`chmod +x main.py\`
4. View your work: \`ls -la robot_ws/src/my_robot/\`

🎯 **Challenge:** Use \`grep\` to find all Python files in your home directory!
`
  },

  {
    id: 4,
    title: "Understanding URDF",
    part: 1,
    partName: "For Students",
    startPage: 13,
    endPage: 15,
    difficulty: "Easy",
    estimatedTime: "20 minutes",
    objectives: [
      "Learn what URDF is and why it's important",
      "Understand links and joints",
      "Create a simple robot description"
    ],
    keywords: ["URDF", "Robot Description", "Links", "Joints", "XML"],
    content: `
# Chapter 4: Understanding URDF

## 🤖 What is URDF?

**URDF** = **Unified Robot Description Format**

It's an **XML file** that describes a robot's:
- Physical structure (links)
- Connections (joints)
- Visual appearance
- Collision geometry
- Sensors and actuators

💡 **Think of URDF as a blueprint for your robot!**

---

## 🏗️ Basic Building Blocks

### 1. Links

A **link** is a rigid body part of the robot.

Examples:
- Base
- Arm segments
- Gripper fingers
- Wheels

\`\`\`xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>
\`\`\`

### 2. Joints

A **joint** connects two links and defines how they move relative to each other.

**Joint Types:**
- \`fixed\` - No movement (welded together)
- \`revolute\` - Rotates (like an elbow)
- \`continuous\` - Rotates 360° (like a wheel)
- \`prismatic\` - Slides (like a telescope)

\`\`\`xml
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
\`\`\`

---

## 📐 Coordinate Systems

### XYZ Axes
- **X-axis**: Forward/Backward (Red)
- **Y-axis**: Left/Right (Green)  
- **Z-axis**: Up/Down (Blue)

### RPY (Roll, Pitch, Yaw)
- **Roll**: Rotation around X-axis
- **Pitch**: Rotation around Y-axis
- **Yaw**: Rotation around Z-axis

\`\`\`
      Z
      |
      |
      +---- Y
     /
    /
   X
\`\`\`

---

## 🤖 Example: Simple Robot Arm

Let's build a 2-link robot arm!

\`\`\`xml
<?xml version="1.0"?>
<robot name="simple_arm">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>
  
  <!-- First Arm Segment -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Joint connecting base to link1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- Second Arm Segment -->
  <link name="link2">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.25"/>
      </geometry>
      <origin xyz="0 0 0.125"/>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Joint connecting link1 to link2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
  
  <!-- End Effector -->
  <link name="gripper">
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Joint to gripper -->
  <joint name="gripper_joint" type="fixed">
    <parent link="link2"/>
    <child link="gripper"/>
    <origin xyz="0 0 0.25"/>
  </joint>
  
</robot>
\`\`\`

---

## 🎨 Visualizing URDF

### Method 1: Check URDF Syntax

\`\`\`bash
check_urdf robot.urdf
\`\`\`

### Method 2: View in RViz

\`\`\`bash
# Launch URDF in RViz
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf
\`\`\`

### Method 3: Joint State Publisher

\`\`\`bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui robot.urdf
\`\`\`

This opens a GUI with sliders to move each joint! 🎮

---

## 🔍 Important URDF Elements

### Visual vs Collision

\`\`\`xml
<!-- What you SEE -->
<visual>
  <geometry>
    <mesh filename="fancy_model.dae"/>
  </geometry>
</visual>

<!-- What is used for COLLISION detection -->
<collision>
  <geometry>
    <box size="0.5 0.3 0.1"/>  <!-- Simpler shape for speed -->
  </geometry>
</collision>
\`\`\`

💡 **Pro Tip:** Use simple shapes for collision geometry to improve simulation performance!

### Inertial Properties (For Simulation)

\`\`\`xml
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.01" ixy="0.0" ixz="0.0"
           iyy="0.01" iyz="0.0"
           izz="0.01"/>
</inertial>
\`\`\`

---

## 🛠️ XACRO: URDF on Steroids

**XACRO** = **XML Macros** - Makes URDF more powerful!

Features:
- Variables
- Math operations
- Macros (reusable components)
- Includes

\`\`\`xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Define properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  
  <!-- Reusable macro for wheels -->
  <xacro:macro name="wheel" params="prefix">
    <link name="\${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="\${wheel_radius}" length="\${wheel_width}"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>
  
  <!-- Use the macro -->
  <xacro:wheel prefix="left"/>
  <xacro:wheel prefix="right"/>
  
</robot>
\`\`\`

Convert XACRO to URDF:
\`\`\`bash
xacro robot.xacro > robot.urdf
\`\`\`

---

## 📚 Summary

✅ URDF = Robot blueprint in XML format
✅ **Links** = Robot body parts
✅ **Joints** = Connections between links
✅ Joint types: fixed, revolute, continuous, prismatic
✅ Use RViz to visualize
✅ XACRO makes URDF more maintainable

**Next:** Chapter 5 - Sensors and Perception

---

## ✍️ Exercise

**Task:** Create a URDF for a simple mobile robot with:
- 1 base link (rectangular body)
- 2 wheels (left and right)
- 1 caster wheel (sphere, fixed joint)
Save as \`my_robot.urdf\` and visualize it in RViz! 🎯
`
  },

  {
    id: 5,
    title: "Sensors and Perception",
    part: 1,
    partName: "For Students",
    startPage: 16,
    endPage: 18,
    difficulty: "Easy",
    estimatedTime: "20 minutes",
    objectives: [
      "Understand different types of robot sensors",
      "Learn how robots perceive their environment",
      "Integrate sensors with ROS2"
    ],
    keywords: ["Sensors", "LiDAR", "Camera", "IMU", "Perception", "RGB-D"],
    content: `
# Chapter 5: Sensors and Perception

## 👁️ How Robots See the World

Robots need **sensors** to perceive their environment, just like humans use eyes, ears, and touch. Without sensors, a robot is blind!

---

## 🔍 Types of Sensors

### 1. Cameras (Vision)

**RGB Cameras** - See color images like human eyes

\`\`\`python
# ROS2 Camera Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Display image
        cv2.imshow("Robot Camera", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
\`\`\`

**Depth Cameras (RGB-D)** - See both color AND distance

Examples: Intel RealSense, Kinect

### 2. LiDAR (Light Detection and Ranging)

Creates 3D point clouds by measuring distance with laser beams.

**Common uses:**
- Obstacle detection
- Mapping environments
- Navigation

\`\`\`python
# ROS2 LiDAR Subscriber
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
    
    def scan_callback(self, msg):
        # Get minimum distance detected
        min_distance = min(msg.ranges)
        
        if min_distance < 0.5:  # 50cm
            self.get_logger().warn(f'Obstacle detected at {min_distance}m!')
\`\`\`

### 3. IMU (Inertial Measurement Unit)

Measures:
- **Acceleration** (movement)
- **Angular velocity** (rotation)
- **Orientation** (tilt)

Essential for balance and navigation!

### 4. Encoders

Measure wheel/joint rotation

**Uses:**
- Track distance traveled
- Monitor joint positions
- Calculate speed

---

## 🤖 Sensor Fusion

Combining data from multiple sensors gives better perception!

\`\`\`
Camera + LiDAR + IMU = Robust Perception
\`\`\`

**Example:** Self-driving cars use:
- Cameras (see lanes, signs)
- LiDAR (detect obstacles)
- IMU (know orientation)
- GPS (know location)

---

## 📚 Summary

- **Cameras** = Robot's eyes
- **LiDAR** = 3D distance measurement
- **IMU** = Balance and orientation
- **Encoders** = Position tracking
- **Sensor fusion** = Combining sensors for better perception

**Next:** Chapter 6 - Robot Kinematics

---

## ✍️ Exercise

**Task:** Create a ROS2 node that subscribes to camera AND LiDAR, and prints when both detect an obstacle!

**Hint:** Use two subscribers in one node!
`
  },

  {
    id: 6,
    title: "Robot Kinematics",
    part: 1,
    partName: "For Students",
    startPage: 19,
    endPage: 21,
    difficulty: "Easy",
    estimatedTime: "25 minutes",
    objectives: [
      "Understand forward and inverse kinematics",
      "Learn coordinate frames and transformations",
      "Calculate robot arm positions"
    ],
    keywords: ["Kinematics", "Forward Kinematics", "Inverse Kinematics", "Transforms", "End Effector"],
    content: `
# Chapter 6: Robot Kinematics

## 🦾 How Robots Move

**Kinematics** = The math of robot motion (without considering forces)

---

## 📐 Two Types of Kinematics

### Forward Kinematics (FK)

**Question:** If I set joint angles to X, Y, Z... where will the end effector be?

**Given:** Joint angles
**Find:** End effector position

\`\`\`
Joint Angles → FK → End Position
\`\`\`

**Example:**
\`\`\`python
# Simple 2-link arm FK
import numpy as np

def forward_kinematics(theta1, theta2, L1, L2):
    """
    theta1, theta2: joint angles (radians)
    L1, L2: link lengths
    Returns: (x, y) end effector position
    """
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    
    return x, y

# Example: 2 joints at 45°, links of 1m each
theta1 = np.pi/4  # 45 degrees
theta2 = np.pi/4
L1, L2 = 1.0, 1.0

x, y = forward_kinematics(theta1, theta2, L1, L2)
print(f"End effector at: ({x:.2f}, {y:.2f})")
\`\`\`

### Inverse Kinematics (IK)

**Question:** To reach position X, Y, Z... what should my joint angles be?

**Given:** Desired end position
**Find:** Required joint angles

\`\`\`
End Position → IK → Joint Angles
\`\`\`

⚠️ **IK is MUCH harder than FK!**

Problems:
- Multiple solutions possible
- Sometimes no solution exists
- Computationally expensive

**Example:**
\`\`\`python
def inverse_kinematics_2link(x, y, L1, L2):
    """
    Returns: (theta1, theta2) joint angles
    """
    # Distance to target
    d = np.sqrt(x**2 + y**2)
    
    # Check if reachable
    if d > (L1 + L2) or d < abs(L1 - L2):
        return None  # Target unreachable!
    
    # Law of cosines
    cos_theta2 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(cos_theta2)
    
    # Calculate theta1
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
    
    return theta1, theta2

# Example: Reach point (1.5, 0.5)
result = inverse_kinematics_2link(1.5, 0.5, 1.0, 1.0)
if result:
    theta1, theta2 = result
    print(f"Solution: θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°")
else:
    print("Target unreachable!")
\`\`\`

---

## 🎯 Real-World Use

### Humanoid Walking

1. **IK**: Calculate leg joint angles to place foot at desired position
2. **FK**: Verify the calculated position is correct

### Robot Arm Pick-and-Place

1. See object location (camera)
2. **IK**: Calculate arm joints to reach it
3. Move arm
4. **FK**: Check if actually reached target

---

## 🔧 In ROS2

ROS2 has packages for kinematics:

\`\`\`bash
# Install MoveIt2 (motion planning framework)
sudo apt install ros-humble-moveit

# MoveIt2 handles IK automatically!
\`\`\`

---

## 📚 Summary

✅ **Forward Kinematics** = Joints → Position (easy!)
✅ **Inverse Kinematics** = Position → Joints (hard!)
✅ IK can have multiple solutions or no solution
✅ MoveIt2 can solve IK for complex robots

**Next:** Chapter 7 - Basic Navigation

---

## ✍️ Exercise

**Task:** Calculate FK for a 3-link arm with:
- θ1 = 0°, θ2 = 90°, θ3 = -45°
- All links are 0.5m long

**Challenge:** Now solve IK to reach point (0.5, 0.5)!
`
  },

  {
    id: 7,
    title: "Basic Navigation",
    part: 1,
    partName: "For Students",
    startPage: 22,
    endPage: 24,
    difficulty: "Easy",
    estimatedTime: "20 minutes",
    objectives: [
      "Understand robot navigation concepts",
      "Learn about path planning",
      "Use Nav2 stack basics"
    ],
    keywords: ["Navigation", "Path Planning", "Nav2", "Obstacle Avoidance", "Localization"],
    content: `
# Chapter 7: Basic Navigation

## 🗺️ How Robots Navigate

**Navigation** = Getting from Point A to Point B safely!

Three key questions:
1. **Where am I?** (Localization)
2. **Where am I going?** (Goal)
3. **How do I get there?** (Path Planning)

---

## 🧭 Components of Navigation

### 1. Localization

Knowing your position in the world

**Methods:**
- GPS (outdoors)
- SLAM (indoors)
- Wheel odometry
- Sensor fusion

### 2. Mapping

Creating a map of the environment

**Types:**
- **Occupancy Grid** - 2D grid of free/occupied spaces
- **Point Cloud** - 3D points from LiDAR
- **Semantic Map** - Labeled objects (door, chair, wall)

### 3. Path Planning

Finding a collision-free path to the goal

**Algorithms:**
- A* (A-star)
- Dijkstra
- RRT (Rapidly-exploring Random Tree)
- DWA (Dynamic Window Approach)

### 4. Obstacle Avoidance

Avoiding dynamic obstacles (people, other robots)

---

## 🤖 Nav2 Stack

**Nav2** = ROS2's navigation framework

\`\`\`bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Install TurtleBot3 simulation
sudo apt install ros-humble-turtlebot3-gazebo
\`\`\`

### Basic Navigation Launch

\`\`\`bash
# Terminal 1: Start simulation
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Start navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

# Terminal 3: Open RViz
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
\`\`\`

### Send Navigation Goal (Python)

\`\`\`python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NavigationGoal(Node):
    def __init__(self):
        super().__init__('navigation_goal')
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        self.send_goal()
    
    def send_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Set goal position
        goal.pose.position.x = 2.0
        goal.pose.position.y = 1.0
        goal.pose.position.z = 0.0
        
        # Set goal orientation (facing forward)
        goal.pose.orientation.w = 1.0
        
        self.publisher.publish(goal)
        self.get_logger().info('Goal sent!')

def main():
    rclpy.init()
    node = NavigationGoal()
    rclpy.spin(node)
\`\`\`

---

## 🚧 Obstacle Avoidance

Nav2 uses **costmaps** - grids showing safe/unsafe areas

**Layers:**
- Static Layer: Walls from map
- Obstacle Layer: From sensors (LiDAR, camera)
- Inflation Layer: Safety margin around obstacles

**Robot stays in low-cost areas!**

---

## 🎯 Path Planning Basics

### A* Algorithm (Simplified)

1. Start at current position
2. Look at neighbors
3. Calculate cost to each neighbor
4. Pick lowest cost
5. Repeat until goal reached

**Cost = Distance + Obstacle Avoidance**

---

## 📚 Summary

✅ Navigation = Localization + Mapping + Path Planning
✅ Nav2 is ROS2's navigation framework
✅ Costmaps help avoid obstacles
✅ A* is a common path planning algorithm

**Next:** Chapter 8 - Simple Manipulation

---

## ✍️ Exercise

**Task:** 
1. Launch TurtleBot3 simulation
2. Use RViz to send a navigation goal
3. Watch the robot navigate!

**Bonus:** Try sending goals through Python code!
`
  },

  {
    id: 8,
    title: "Simple Manipulation",
    part: 1,
    partName: "For Students",
    startPage: 25,
    endPage: 27,
    difficulty: "Easy",
    estimatedTime: "20 minutes",
    objectives: [
      "Understand robot manipulation basics",
      "Learn about grippers and end effectors",
      "Control a simple robotic arm"
    ],
    keywords: ["Manipulation", "Gripper", "Pick and Place", "End Effector", "Grasping"],
    content: `
# Chapter 8: Simple Manipulation

## 🦾 Robot Arms and Grippers

**Manipulation** = Using robot arms to interact with objects

---

## 🔧 End Effectors

The "hand" of a robot - attached to the end of the arm

**Types:**
1. **Grippers** - Two or more fingers to grasp
2. **Suction Cups** - Vacuum grip for flat objects
3. **Magnetic** - For metal objects
4. **Specialized** - Welders, spray guns, etc.

---

## ✋ Gripper Control

### Simple Parallel Gripper

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.publisher = self.create_publisher(
            Float64,
            '/gripper/command',
            10
        )
    
    def open_gripper(self):
        msg = Float64()
        msg.data = 0.0  # 0 = fully open
        self.publisher.publish(msg)
        self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        msg = Float64()
        msg.data = 1.0  # 1 = fully closed
        self.publisher.publish(msg)
        self.get_logger().info('Gripper closed')

def main():
    rclpy.init()
    node = GripperControl()
    
    # Open, wait, close
    node.open_gripper()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.close_gripper()
    
    rclpy.shutdown()
\`\`\`

---

## 📦 Pick and Place

**Basic Steps:**
1. **Perceive** - See the object (camera)
2. **Plan** - Calculate path to object (IK)
3. **Approach** - Move arm above object
4. **Grasp** - Close gripper
5. **Lift** - Move arm up
6. **Move** - Transport to target
7. **Release** - Open gripper
8. **Retreat** - Move arm away

### Example Pick and Place

\`\`\`python
class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        # ... setup arm and gripper controllers ...
    
    def execute_pick_and_place(self, object_pos, target_pos):
        # 1. Move above object
        above_pos = [object_pos[0], object_pos[1], object_pos[2] + 0.1]
        self.move_arm(above_pos)
        
        # 2. Open gripper
        self.open_gripper()
        
        # 3. Move down to object
        self.move_arm(object_pos)
        
        # 4. Close gripper
        self.close_gripper()
        time.sleep(1.0)  # Wait for grasp
        
        # 5. Lift object
        self.move_arm(above_pos)
        
        # 6. Move to target
        above_target = [target_pos[0], target_pos[1], target_pos[2] + 0.1]
        self.move_arm(above_target)
        
        # 7. Lower to target
        self.move_arm(target_pos)
        
        # 8. Release
        self.open_gripper()
        
        # 9. Retreat
        self.move_arm(above_target)
        
        self.get_logger().info('Pick and place complete!')
\`\`\`

---

## 🎯 Grasping Strategies

### Force Control

Grip just tight enough - not too loose, not too tight!

\`\`\`python
def adaptive_grasp(self, force_sensor):
    target_force = 5.0  # Newtons
    current_force = force_sensor.read()
    
    while current_force < target_force:
        self.close_gripper(increment=0.1)
        current_force = force_sensor.read()
        
        if current_force > target_force * 1.2:
            self.get_logger().warn('Grasping too hard!')
            break
\`\`\`

### Vision-Based Grasping

1. Detect object with camera
2. Estimate object pose
3. Calculate grasp points
4. Execute grasp

---

## 🤖 MoveIt2 for Manipulation

**MoveIt2** handles complex manipulation:

\`\`\`bash
# Install MoveIt2
sudo apt install ros-humble-moveit
\`\`\`

**Features:**
- Motion planning
- Collision avoidance
- IK solving
- Path smoothing

\`\`\`python
# MoveIt2 Python API example
from moveit_py import MoveItPy

def move_to_pose(target_pose):
    moveit = MoveItPy()
    arm = moveit.get_planning_component("arm")
    
    # Set target
    arm.set_pose_target(target_pose)
    
    # Plan path
    plan = arm.plan()
    
    # Execute if valid
    if plan:
        arm.execute(plan)
\`\`\`

---

## 📚 Summary

✅ End effectors = Robot "hands"
✅ Grippers use open/close commands
✅ Pick and place = 9-step process
✅ Force control prevents damage
✅ MoveIt2 simplifies manipulation

**Next:** Chapter 9 - Debugging Your Robot

---

## ✍️ Exercise

**Task:** Write a program that:
1. Opens gripper
2. Waits 2 seconds
3. Closes gripper
4. Waits 2 seconds
5. Repeat 5 times

**Bonus:** Add force sensing to detect when object is grasped!
`
  },

  {
    id: 9,
    title: "Debugging Your Robot",
    part: 1,
    partName: "For Students",
    startPage: 28,
    endPage: 29,
    difficulty: "Easy",
    estimatedTime: "15 minutes",
    objectives: [
      "Learn common robot debugging techniques",
      "Use ROS2 debugging tools",
      "Troubleshoot typical problems"
    ],
    keywords: ["Debugging", "RViz", "rqt", "Logging", "Troubleshooting"],
    content: `
# Chapter 9: Debugging Your Robot

## 🐛 Finding and Fixing Problems

Every roboticist spends time debugging! Here's how to do it efficiently.

---

## 🔍 Common Problems

### Problem 1: "My robot won't move!"

**Checklist:**
- ✅ Is the robot powered on?
- ✅ Are motors enabled?
- ✅ Are you sending commands to correct topic?
- ✅ Check message types match

\`\`\`bash
# Check if topic exists
ros2 topic list

# Check topic type
ros2 topic info /cmd_vel

# Echo messages on topic
ros2 topic echo /cmd_vel
\`\`\`

### Problem 2: "Sensors aren't working!"

\`\`\`bash
# List all nodes
ros2 node list

# Check if sensor node is running
ros2 node info /camera_node

# Verify sensor publishing
ros2 topic hz /camera/image_raw
\`\`\`

### Problem 3: "Transforms are broken!"

\`\`\`bash
# View TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo base_link camera_link
\`\`\`

---

## 🛠️ Essential Debugging Tools

### 1. RViz (Visualization)

**The robot debugger's best friend!**

\`\`\`bash
ros2 run rviz2 rviz2
\`\`\`

**What to visualize:**
- Robot model (URDF)
- Sensor data (camera, LiDAR)
- Transforms (TF)
- Paths and trajectories
- Point clouds

💡 **Pro Tip:** Save your RViz config file!

\`\`\`bash
# Save configuration
File → Save Config As → my_config.rviz

# Load configuration
rviz2 -d my_config.rviz
\`\`\`

### 2. rqt Tools

**GUI toolbox for ROS2**

\`\`\`bash
# All-in-one interface
rqt

# Specific tools:
rqt_graph      # Node connection graph
rqt_plot       # Real-time plotting
rqt_console    # Log viewer
rqt_bag        # Bag file playback
\`\`\`

### 3. Command Line Tools

\`\`\`bash
# Node information
ros2 node list
ros2 node info /my_node

# Topic monitoring
ros2 topic list
ros2 topic echo /my_topic
ros2 topic hz /my_topic    # Message frequency

# Service calls
ros2 service list
ros2 service call /my_service std_srvs/srv/Empty

# Parameter inspection
ros2 param list
ros2 param get /my_node my_parameter
\`\`\`

---

## 📊 Logging Best Practices

### Use Proper Log Levels

\`\`\`python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
    
    def process_data(self, data):
        # DEBUG: Detailed information
        self.get_logger().debug(f'Processing: {data}')
        
        # INFO: Normal operation
        self.get_logger().info('Data processed successfully')
        
        # WARN: Something unusual but not critical
        if data < 0:
            self.get_logger().warn('Received negative value')
        
        # ERROR: Something went wrong
        if data is None:
            self.get_logger().error('Received None value!')
        
        # FATAL: Critical failure
        if not self.is_initialized:
            self.get_logger().fatal('Node not initialized!')
\`\`\`

### Set Log Level

\`\`\`bash
# Run with debug logging
ros2 run my_package my_node --ros-args --log-level DEBUG

# Set in code
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
\`\`\`

---

## 🎯 Debugging Workflow

1. **Reproduce** - Make problem happen consistently
2. **Isolate** - Find which component is failing
3. **Inspect** - Use tools to examine state
4. **Hypothesis** - Guess what's wrong
5. **Test** - Try a fix
6. **Verify** - Confirm it's fixed

---

## 💡 Pro Tips

### Record Data for Later Analysis

\`\`\`bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image /scan

# Playback recording
ros2 bag play my_recording.bag
\`\`\`

### Use Launch Files for Complex Systems

\`\`\`python
# launch/debug.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            output='screen',  # Print to console
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config/debug.rviz']
        )
    ])
\`\`\`

---

## 📚 Summary

✅ Check basics first (power, connections, topics)
✅ RViz is essential for visualization
✅ rqt provides many debugging tools
✅ Use proper log levels
✅ Record data for analysis

**Next:** Chapter 10 - Your First Robot Project

---

## ✍️ Exercise

**Task:** Debug this broken code:

\`\`\`python
class BrokenNode(Node):
    def __init__(self):
        super().__init__('broken')
        self.publisher = self.create_publisher(Float64, '/data')
        self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        msg = Float64()
        msg.data = "hello"  # Bug!
        self.publisher.publish(msg)
\`\`\`

**Hint:** Multiple bugs here! Find them all!
`
  },

  {
    id: 10,
    title: "Your First Robot Project",
    part: 1,
    partName: "For Students",
    startPage: 30,
    endPage: 31,
    difficulty: "Easy",
    estimatedTime: "30 minutes",
    objectives: [
      "Integrate all learned concepts",
      "Build a complete robot system",
      "Deploy and test your robot"
    ],
    keywords: ["Project", "Integration", "Robot System", "Deployment", "Testing"],
    content: `
# Chapter 10: Your First Robot Project

## 🎯 Build a Complete Robot!

Time to put everything together! We'll build a **simple delivery robot**.

---

## 🤖 Project: Table Delivery Robot

**Goal:** Robot navigates to table, picks up object, delivers to person

**What You'll Use:**
- ROS2 (framework)
- Navigation (Nav2)
- Manipulation (arm + gripper)
- Sensors (camera, LiDAR)
- Everything from Chapters 1-9!

---

## 📋 System Architecture

\`\`\`
[Camera] ──→ [Vision Node] ──→ [Object Detector]
                                      ↓
[LiDAR]  ──→ [Nav2]        ──→ [Path Planner]
                                      ↓
[IMU]    ──→ [Localization] ──→ [State Machine] ──→ [Robot Control]
                                      ↑
[Gripper] ←─ [Manipulation Node] ←──┘
\`\`\`

---

## 🔧 Implementation

### Step 1: Setup Workspace

\`\`\`bash
mkdir -p ~/delivery_robot_ws/src
cd ~/delivery_robot_ws/src

# Create package
ros2 pkg create delivery_robot \\
    --build-type ament_python \\
    --dependencies rclpy geometry_msgs sensor_msgs

cd ~/delivery_robot_ws
colcon build
source install/setup.bash
\`\`\`

### Step 2: Main State Machine

\`\`\`python
# src/delivery_robot/state_machine.py
import rclpy
from rclpy.node import Node
from enum import Enum

class State(Enum):
    IDLE = 0
    NAVIGATE_TO_TABLE = 1
    DETECT_OBJECT = 2
    PICK_OBJECT = 3
    NAVIGATE_TO_PERSON = 4
    DELIVER_OBJECT = 5
    RETURN_HOME = 6

class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('delivery_robot')
        self.state = State.IDLE
        self.create_timer(0.1, self.state_machine)
        
        # Initialize sub-systems
        self.navigation = NavigationController(self)
        self.manipulator = ManipulationController(self)
        self.vision = VisionDetector(self)
    
    def state_machine(self):
        if self.state == State.IDLE:
            self.get_logger().info('Waiting for command...')
            # Wait for start command
        
        elif self.state == State.NAVIGATE_TO_TABLE:
            self.get_logger().info('Navigating to table')
            if self.navigation.go_to_pose(table_pose):
                self.state = State.DETECT_OBJECT
        
        elif self.state == State.DETECT_OBJECT:
            self.get_logger().info('Looking for object')
            if self.vision.detect_object():
                self.state = State.PICK_OBJECT
        
        elif self.state == State.PICK_OBJECT:
            self.get_logger().info('Picking up object')
            if self.manipulator.pick_object():
                self.state = State.NAVIGATE_TO_PERSON
        
        elif self.state == State.NAVIGATE_TO_PERSON:
            self.get_logger().info('Delivering to person')
            if self.navigation.go_to_pose(person_pose):
                self.state = State.DELIVER_OBJECT
        
        elif self.state == State.DELIVER_OBJECT:
            self.get_logger().info('Releasing object')
            if self.manipulator.release_object():
                self.state = State.RETURN_HOME
        
        elif self.state == State.RETURN_HOME:
            self.get_logger().info('Returning home')
            if self.navigation.go_to_pose(home_pose):
                self.state = State.IDLE
                self.get_logger().info('Mission complete!')
\`\`\`

### Step 3: Launch File

\`\`\`python
# launch/delivery_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Main state machine
        Node(
            package='delivery_robot',
            executable='state_machine',
            output='screen'
        ),
        
        # Navigation
        Node(
            package='nav2_bringup',
            executable='navigation',
            parameters=['config/nav2_params.yaml']
        ),
        
        # Vision
        Node(
            package='delivery_robot',
            executable='vision_detector'
        ),
        
        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'config/delivery.rviz']
        )
    ])
\`\`\`

---

## 🧪 Testing

### Test Individual Components

\`\`\`bash
# Test navigation only
ros2 launch delivery_robot test_navigation.launch.py

# Test manipulation only
ros2 launch delivery_robot test_manipulation.launch.py

# Test vision only
ros2 launch delivery_robot test_vision.launch.py
\`\`\`

### Integration Test

\`\`\`bash
# Run complete system
ros2 launch delivery_robot delivery_robot.launch.py

# Send start command
ros2 topic pub /start_delivery std_msgs/msg/Empty "{}"
\`\`\`

---

## 📊 Success Criteria

✅ Robot navigates to table
✅ Detects object with camera
✅ Successfully picks object
✅ Navigates to person
✅ Delivers object safely
✅ Returns to start position

---

## 🎓 You Did It!

**Congratulations!** You've completed Part 1: For Students!

You now know:
- ✅ ROS2 basics
- ✅ Linux commands
- ✅ Robot description (URDF)
- ✅ Sensors and perception
- ✅ Kinematics
- ✅ Navigation
- ✅ Manipulation
- ✅ Debugging
- ✅ System integration

---

## 🚀 Next Steps

**Part 2: For Researchers** covers:
- Advanced simulations (Gazebo, Isaac Sim)
- Computer vision
- Reinforcement learning
- Research methodologies

**Ready to level up?** → Chapter 11!

---

## ✍️ Final Project Ideas

1. **Cleaning Robot** - Navigate and vacuum
2. **Security Bot** - Patrol and detect intruders
3. **Waiter Robot** - Serve food in restaurant
4. **Agricultural Bot** - Harvest crops
5. **Warehouse Robot** - Sort and move packages

**Build something awesome!** 🤖✨
`
  },

  // ==========================================
  // PART 2: FOR RESEARCHERS (12 Chapters, Pages 32-61)
  // ==========================================

  {
    id: 11,
    title: "Advanced Simulation with Gazebo",
    part: 2,
    partName: "For Researchers",
    startPage: 32,
    endPage: 34,
    difficulty: "Medium",
    estimatedTime: "35 minutes",
    objectives: [
      "Master Gazebo simulation environment",
      "Create custom worlds and models",
      "Simulate sensors and physics accurately"
    ],
    keywords: ["Gazebo", "Simulation", "Physics Engine", "World Building", "SDF"],
    content: `
# Chapter 11: Advanced Simulation with Gazebo

## 🌍 Building Virtual Worlds

**Gazebo** is the industry-standard robot simulator. Perfect for testing before deploying to real hardware!

---

## 🏗️ Creating Custom Worlds

### World File Structure (SDF)

\`\`\`xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="research_lab">
    <!-- Physics settings -->
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    
    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
      </link>
    </model>
    
    <!-- Custom obstacles -->
    <model name="wall">
      <pose>5 0 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
\`\`\`

---

## 🤖 Spawning Robots Programmatically

\`\`\`python
import rclpy
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import os

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
    
    def spawn_robot(self, robot_name, urdf_path, x, y, z):
        with open(urdf_path, 'r') as f:
            robot_xml = f.read()
        
        request = SpawnEntity.Request()
        request.name = robot_name
        request.xml = robot_xml
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'{robot_name} spawned successfully!')
        else:
            self.get_logger().error(f'Failed to spawn {robot_name}')

# Usage
spawner = RobotSpawner()
spawner.spawn_robot('my_robot', 'robot.urdf', 0.0, 0.0, 0.5)
\`\`\`

---

## 📡 Simulating Sensors

### Camera Plugin

\`\`\`xml
<sensor name="camera" type="camera">
  <update_rate>30.0</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>camera/image_raw:=camera/image</remapping>
    </ros>
  </plugin>
</sensor>
\`\`\`

### LiDAR Plugin

\`\`\`xml
<sensor name="lidar" type="ray">
  <pose>0 0 0.1 0 0 0</pose>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <update_rate>10</update_rate>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
\`\`\`

---

## ⚙️ Physics Tuning

### Contact Properties

\`\`\`xml
<gazebo>
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    
    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>
    
    <!-- Solver settings -->
    <ode>
      <solver>
        <type>quick</type>
        <iters>50</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
</gazebo>
\`\`\`

---

## 🎬 Recording and Playback

\`\`\`bash
# Record simulation
gazebo --verbose --record_path=/tmp/sim_recording

# Playback recorded session
gazebo --play /tmp/sim_recording/state.log
\`\`\`

---

## 📚 Summary

✅ Create custom worlds with SDF format
✅ Spawn robots programmatically
✅ Simulate realistic sensors with noise
✅ Tune physics for accuracy
✅ Record and replay simulations

**Next:** Chapter 12 - Computer Vision for Robotics
`
  },

  {
    id: 12,
    title: "Computer Vision for Robotics",
    part: 2,
    partName: "For Researchers",
    startPage: 35,
    endPage: 37,
    difficulty: "Medium",
    estimatedTime: "40 minutes",
    objectives: [
      "Implement object detection and tracking",
      "Use deep learning for vision tasks",
      "Integrate vision with robot control"
    ],
    keywords: ["Computer Vision", "OpenCV", "YOLO", "Object Detection", "Tracking"],
    content: `
# Chapter 12: Computer Vision for Robotics

## 👁️ Teaching Robots to See

Computer vision enables robots to understand their visual environment.

---

## 📷 Image Processing with OpenCV

### Basic Image Operations

\`\`\`python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process image
        processed = self.detect_objects(cv_image)
        
        # Convert back to ROS Image
        ros_image = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
        self.publisher.publish(ros_image)
    
    def detect_objects(self, image):
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define red color range
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        
        # Create mask
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Draw bounding boxes
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Minimum area
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(image, 'Red Object', (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return image
\`\`\`

---

## 🎯 Object Detection with YOLO

### Using YOLOv8

\`\`\`python
from ultralytics import YOLO
import cv2

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLOv8 model
        self.model = YOLO('yolov8n.pt')  # nano model
        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.detect_callback, 10
        )
        self.publisher = self.create_publisher(
            DetectionArray, '/detections', 10
        )
        self.bridge = CvBridge()
    
    def detect_callback(self, msg):
        # Convert to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run detection
        results = self.model(image, conf=0.5)  # 50% confidence threshold
        
        detections = DetectionArray()
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get detection info
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.model.names[cls]
                
                # Create detection message
                detection = Detection()
                detection.label = label
                detection.confidence = conf
                detection.bbox = [int(x1), int(y1), int(x2), int(y2)]
                
                detections.detections.append(detection)
                
                # Draw on image
                cv2.rectangle(image, (int(x1), int(y1)), 
                            (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(image, f'{label} {conf:.2f}',
                          (int(x1), int(y1)-10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        self.publisher.publish(detections)
        cv2.imshow('Detections', image)
        cv2.waitKey(1)
\`\`\`

---

## 🎭 Object Tracking

### CSRT Tracker

\`\`\`python
class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.tracker = cv2.TrackerCSRT_create()
        self.tracking = False
        self.bbox = None
        
    def initialize_tracker(self, image, bbox):
        """Initialize tracker with bounding box"""
        self.tracker.init(image, bbox)
        self.tracking = True
    
    def update_tracker(self, image):
        """Update tracker with new frame"""
        if not self.tracking:
            return None
        
        success, bbox = self.tracker.update(image)
        
        if success:
            # Draw tracked object
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            return bbox
        else:
            self.tracking = False
            self.get_logger().warn('Tracking failed!')
            return None
\`\`\`

---

## 🧮 3D Vision with Depth Cameras

### Point Cloud Processing

\`\`\`python
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2, '/camera/depth/points', 
            self.pointcloud_callback, 10
        )
    
    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 to numpy array
        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            points.append([point[0], point[1], point[2]])
        
        points = np.array(points)
        
        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Downsample
        pcd_down = pcd.voxel_down_sample(voxel_size=0.02)
        
        # Plane segmentation (find ground)
        plane_model, inliers = pcd_down.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )
        
        # Extract objects (non-ground points)
        objects = pcd_down.select_by_index(inliers, invert=True)
        
        # Cluster objects
        labels = np.array(objects.cluster_dbscan(
            eps=0.05, min_points=10
        ))
        
        max_label = labels.max()
        self.get_logger().info(f'Found {max_label + 1} objects')
\`\`\`

---

## 🎨 Visual Servoing

### Image-Based Visual Servoing (IBVS)

\`\`\`python
class VisualServoing(Node):
    def __init__(self):
        super().__init__('visual_servoing')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # PID gains
        self.kp = 0.002
        self.ki = 0.0001
        self.kd = 0.001
        
        self.error_sum = 0
        self.last_error = 0
    
    def servo_to_target(self, target_center, image_center):
        """
        Move robot to center object in camera view
        """
        # Calculate error
        error_x = target_center[0] - image_center[0]
        error_y = target_center[1] - image_center[1]
        
        # PID control
        self.error_sum += error_x
        error_diff = error_x - self.last_error
        
        # Calculate control signal
        control = (self.kp * error_x + 
                  self.ki * self.error_sum + 
                  self.kd * error_diff)
        
        # Create velocity command
        cmd = Twist()
        cmd.angular.z = control  # Turn to center object
        cmd.linear.x = 0.2 if abs(error_x) < 50 else 0.0  # Move forward when centered
        
        self.cmd_pub.publish(cmd)
        self.last_error = error_x
\`\`\`

---

## 📚 Summary

✅ Process images with OpenCV
✅ Detect objects with YOLO
✅ Track objects across frames
✅ Process 3D point clouds
✅ Implement visual servoing

**Next:** Chapter 13 - SLAM and Mapping
`
  },

  {
    id: 13,
    title: "SLAM and Mapping",
    part: 2,
    partName: "For Researchers",
    startPage: 38,
    endPage: 40,
    difficulty: "Medium",
    estimatedTime: "45 minutes",
    objectives: [
      "Understand SLAM algorithms",
      "Implement mapping systems",
      "Use Cartographer and SLAM Toolbox"
    ],
    keywords: ["SLAM", "Mapping", "Cartographer", "Localization", "Occupancy Grid"],
    content: `
# Chapter 13: SLAM and Mapping

## 🗺️ Simultaneous Localization and Mapping

**SLAM** = Building a map while tracking your position in it!

---

## 🎯 The SLAM Problem

**Challenge:** You need a map to localize, but you need localization to build a map!

**Solution:** Do both simultaneously!

---

## 📊 Types of SLAM

### 1. LiDAR SLAM

Uses laser scans for 2D/3D mapping

**Popular Algorithms:**
- Cartographer (Google)
- SLAM Toolbox
- Hector SLAM
- GMapping

### 2. Visual SLAM (vSLAM)

Uses cameras for mapping

**Popular Algorithms:**
- ORB-SLAM3
- RTAB-Map
- LSD-SLAM

### 3. RGB-D SLAM

Uses depth cameras (color + depth)

---

## 🚀 Using Cartographer

### Installation

\`\`\`bash
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
\`\`\`

### Configuration File

\`\`\`lua
-- cartographer.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true

MAP_BUILDER.use_trajectory_builder_2d = true

return options
\`\`\`

### Launch Cartographer

\`\`\`python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            parameters=[{
                'use_sim_time': True
            }],
            arguments=[
                '-configuration_directory', '/path/to/config',
                '-configuration_basename', 'cartographer.lua'
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu')
            ]
        ),
        
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05
            }]
        )
    ])
\`\`\`

---

## 🛠️ SLAM Toolbox

### Lifelong Mapping

\`\`\`yaml
# slam_toolbox_params.yaml
slam_toolbox:
  ros__parameters:
    # Solver Parameters
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
    
    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping
    
    # Scan matching
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    
    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
\`\`\`

---

## 📍 Localization in Known Map

### AMCL (Adaptive Monte Carlo Localization)

\`\`\`yaml
# amcl_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    
    # Filter parameters
    min_particles: 500
    max_particles: 2000
    
    # Odometry model
    odom_model_type: "diff-corrected"
    odom_alpha1: 0.2
    odom_alpha2: 0.2
    odom_alpha3: 0.2
    odom_alpha4: 0.2
    
    # Laser model
    laser_model_type: "likelihood_field"
    laser_max_range: 10.0
    laser_min_range: 0.1
    
    # Update frequencies
    update_min_d: 0.25
    update_min_a: 0.2
    
    # Initial pose
    set_initial_pose: true
    initial_pose_x: 0.0
    initial_pose_y: 0.0
    initial_pose_a: 0.0
\`\`\`

---

## 🧮 Implementing Simple SLAM

### Particle Filter SLAM (Basic)

\`\`\`python
import numpy as np

class ParticleFilterSLAM:
    def __init__(self, num_particles=100):
        self.num_particles = num_particles
        
        # Initialize particles [x, y, theta]
        self.particles = np.random.rand(num_particles, 3)
        self.particles[:, 0] *= 10  # x: 0-10m
        self.particles[:, 1] *= 10  # y: 0-10m
        self.particles[:, 2] *= 2 * np.pi  # theta: 0-2π
        
        # Particle weights
        self.weights = np.ones(num_particles) / num_particles
        
        # Map (occupancy grid)
        self.map = np.zeros((100, 100))  # 10x10m map with 0.1m resolution
    
    def motion_update(self, delta_x, delta_y, delta_theta):
        """Update particles based on odometry"""
        # Add noise to motion
        noise_x = np.random.normal(0, 0.1, self.num_particles)
        noise_y = np.random.normal(0, 0.1, self.num_particles)
        noise_theta = np.random.normal(0, 0.05, self.num_particles)
        
        # Update particle poses
        self.particles[:, 0] += delta_x + noise_x
        self.particles[:, 1] += delta_y + noise_y
        self.particles[:, 2] += delta_theta + noise_theta
        
        # Normalize angles
        self.particles[:, 2] = np.arctan2(
            np.sin(self.particles[:, 2]),
            np.cos(self.particles[:, 2])
        )
    
    def measurement_update(self, scan_data):
        """Update particle weights based on laser scan"""
        for i, particle in enumerate(self.particles):
            # Calculate likelihood of scan given particle pose
            likelihood = self.calculate_scan_likelihood(
                particle, scan_data
            )
            self.weights[i] *= likelihood
        
        # Normalize weights
        self.weights /= np.sum(self.weights)
        
        # Resample if needed
        if 1.0 / np.sum(self.weights**2) < self.num_particles / 2:
            self.resample()
    
    def resample(self):
        """Resample particles based on weights"""
        indices = np.random.choice(
            self.num_particles,
            size=self.num_particles,
            p=self.weights
        )
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def get_pose_estimate(self):
        """Get best pose estimate (weighted mean)"""
        pose = np.average(self.particles, weights=self.weights, axis=0)
        return pose
\`\`\`

---

## 📚 Summary

✅ SLAM builds map while localizing
✅ Cartographer for production SLAM
✅ SLAM Toolbox for lifelong mapping
✅ AMCL for localization in known maps
✅ Particle filters are fundamental to SLAM

**Next:** Chapter 14 - Motion Planning Algorithms
`
  },

  {
    id: 14,
    title: "Motion Planning Algorithms",
    part: 2,
    partName: "For Researchers",
    startPage: 41,
    endPage: 43,
    difficulty: "Medium",
    estimatedTime: "40 minutes",
    objectives: [
      "Implement sampling-based planners",
      "Use optimization-based planning",
      "Compare planning algorithms"
    ],
    keywords: ["Motion Planning", "RRT", "A*", "Trajectory Optimization", "Collision Avoidance"],
    content: `
# Chapter 14: Motion Planning Algorithms

## 🎯 Finding Collision-Free Paths

Motion planning finds paths from start to goal while avoiding obstacles.

---

## 📊 Categories of Planning

### 1. **Graph Search** (Discrete)
- A*, Dijkstra
- Fast, complete
- Requires discretized space

### 2. **Sampling-Based** (Continuous)
- RRT, RRT*
- Probabilistically complete
- Works in high dimensions

### 3. **Optimization-Based**
- TEB, DWA
- Optimal trajectories
- Considers dynamics

---

## 🌳 RRT (Rapidly-exploring Random Tree)

### Basic RRT Implementation

\`\`\`python
import numpy as np
import matplotlib.pyplot as plt

class RRT:
    def __init__(self, start, goal, obstacles, bounds):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.bounds = bounds
        
        self.tree = [self.start]
        self.parent = {tuple(self.start): None}
        
        self.step_size = 0.5
        self.goal_sample_rate = 0.1
        self.max_iterations = 5000
    
    def sample_point(self):
        """Sample random point in space"""
        if np.random.random() < self.goal_sample_rate:
            return self.goal
        
        return np.array([
            np.random.uniform(self.bounds[0][0], self.bounds[0][1]),
            np.random.uniform(self.bounds[1][0], self.bounds[1][1])
        ])
    
    def nearest_node(self, point):
        """Find nearest node in tree"""
        distances = [np.linalg.norm(point - node) for node in self.tree]
        return self.tree[np.argmin(distances)]
    
    def steer(self, from_node, to_point):
        """Steer from node toward point"""
        direction = to_point - from_node
        distance = np.linalg.norm(direction)
        
        if distance < self.step_size:
            return to_point
        
        return from_node + (direction / distance) * self.step_size
    
    def is_collision_free(self, from_node, to_node):
        """Check if path is collision-free"""
        for obstacle in self.obstacles:
            if self.line_circle_collision(from_node, to_node, 
                                         obstacle['center'], 
                                         obstacle['radius']):
                return False
        return True
    
    def line_circle_collision(self, p1, p2, circle_center, radius):
        """Check line-circle collision"""
        d = p2 - p1
        f = p1 - circle_center
        
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius**2
        
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return False
        
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)
        
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)
    
    def plan(self):
        """Execute RRT planning"""
        for i in range(self.max_iterations):
            # Sample random point
            random_point = self.sample_point()
            
            # Find nearest node
            nearest = self.nearest_node(random_point)
            
            # Steer toward sampled point
            new_node = self.steer(nearest, random_point)
            
            # Check collision
            if self.is_collision_free(nearest, new_node):
                self.tree.append(new_node)
                self.parent[tuple(new_node)] = tuple(nearest)
                
                # Check if goal reached
                if np.linalg.norm(new_node - self.goal) < self.step_size:
                    self.tree.append(self.goal)
                    self.parent[tuple(self.goal)] = tuple(new_node)
                    return self.extract_path()
        
        return None  # No path found
    
    def extract_path(self):
        """Extract path from tree"""
        path = [self.goal]
        current = tuple(self.goal)
        
        while current is not None:
            current = self.parent[current]
            if current is not None:
                path.append(np.array(current))
        
        return path[::-1]  # Reverse path

# Usage
obstacles = [
    {'center': np.array([2, 2]), 'radius': 0.5},
    {'center': np.array([4, 3]), 'radius': 0.7},
]

rrt = RRT(
    start=[0, 0],
    goal=[5, 5],
    obstacles=obstacles,
    bounds=[[0, 6], [0, 6]]
)

path = rrt.plan()
if path:
    print(f"Path found with {len(path)} waypoints")
\`\`\`

---

## ⭐ A* Algorithm

### Implementation

\`\`\`python
import heapq

class AStar:
    def __init__(self, grid, start, goal):
        self.grid = grid  # 0 = free, 1 = obstacle
        self.start = start
        self.goal = goal
        
        self.rows, self.cols = grid.shape
    
    def heuristic(self, pos):
        """Euclidean distance heuristic"""
        return np.sqrt(
            (pos[0] - self.goal[0])**2 + 
            (pos[1] - self.goal[1])**2
        )
    
    def get_neighbors(self, pos):
        """Get valid neighbors (8-connected)"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = pos[0] + dx, pos[1] + dy
                
                # Check bounds
                if 0 <= nx < self.rows and 0 <= ny < self.cols:
                    # Check if free
                    if self.grid[nx, ny] == 0:
                        neighbors.append((nx, ny))
        
        return neighbors
    
    def plan(self):
        """Execute A* planning"""
        # Priority queue: (f_score, g_score, position)
        open_set = [(self.heuristic(self.start), 0, self.start)]
        
        came_from = {}
        g_score = {self.start: 0}
        
        while open_set:
            _, current_g, current = heapq.heappop(open_set)
            
            # Goal reached
            if current == self.goal:
                return self.reconstruct_path(came_from, current)
            
            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                # Calculate tentative g_score
                tentative_g = current_g + 1  # Assuming unit cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor)
                    
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
        
        return None  # No path found
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dict"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]
\`\`\`

---

## 🚀 Dynamic Window Approach (DWA)

### For Dynamic Obstacles

\`\`\`python
class DWA:
    def __init__(self):
        self.max_speed = 1.0
        self.max_yaw_rate = 1.0
        self.max_accel = 0.5
        self.max_dyaw_rate = 0.5
        
        self.v_resolution = 0.1
        self.yaw_rate_resolution = 0.1
        
        self.dt = 0.1
        self.predict_time = 3.0
    
    def calc_dynamic_window(self, current_v, current_yaw_rate):
        """Calculate dynamic window based on current velocity"""
        # Min/max velocities considering acceleration limits
        min_v = max(0, current_v - self.max_accel * self.dt)
        max_v = min(self.max_speed, current_v + self.max_accel * self.dt)
        
        min_yaw = max(-self.max_yaw_rate, 
                     current_yaw_rate - self.max_dyaw_rate * self.dt)
        max_yaw = min(self.max_yaw_rate,
                     current_yaw_rate + self.max_dyaw_rate * self.dt)
        
        return [min_v, max_v, min_yaw, max_yaw]
    
    def predict_trajectory(self, x, y, theta, v, yaw_rate):
        """Predict trajectory for given velocity"""
        trajectory = []
        time = 0
        
        while time <= self.predict_time:
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += yaw_rate * self.dt
            time += self.dt
            
            trajectory.append([x, y, theta])
        
        return np.array(trajectory)
    
    def calc_cost(self, trajectory, goal, obstacles):
        """Calculate trajectory cost"""
        # Heading cost (distance to goal heading)
        goal_heading = np.arctan2(goal[1] - trajectory[-1][1],
                                  goal[0] - trajectory[-1][0])
        heading_cost = abs(goal_heading - trajectory[-1][2])
        
        # Distance cost (distance to goal)
        dist_cost = np.linalg.norm(trajectory[-1][:2] - goal[:2])
        
        # Obstacle cost
        min_dist = float('inf')
        for point in trajectory:
            for obs in obstacles:
                dist = np.linalg.norm(point[:2] - obs[:2])
                min_dist = min(min_dist, dist)
        
        if min_dist < 0.3:  # Safety threshold
            obs_cost = float('inf')
        else:
            obs_cost = 1.0 / min_dist
        
        # Total cost (weighted sum)
        total_cost = (0.5 * heading_cost + 
                     0.3 * dist_cost + 
                     0.2 * obs_cost)
        
        return total_cost
\`\`\`

---

## 📚 Summary

✅ RRT explores space randomly
✅ A* optimal for grid-based planning
✅ DWA handles dynamic obstacles
✅ Each algorithm has trade-offs
✅ Combine multiple planners for robustness

**Next:** Chapter 15 - Reinforcement Learning for Robots
`
  },

  {
    id: 15,
    title: "Reinforcement Learning for Robots",
    part: 2,
    partName: "For Researchers",
    startPage: 44,
    endPage: 46,
    difficulty: "Medium",
    estimatedTime: "50 minutes",
    objectives: [
      "Understand RL fundamentals for robotics",
      "Implement Q-Learning and Policy Gradients",
      "Train robots in simulation"
    ],
    keywords: ["Reinforcement Learning", "Q-Learning", "Policy Gradient", "Reward Function", "Training"],
    content: `
# Chapter 15: Reinforcement Learning for Robots

## 🎮 Teaching Robots Through Rewards

**Reinforcement Learning (RL)** = Learning by trial and error with rewards/penalties

---

## 🧠 RL Fundamentals

### Key Concepts

**Agent:** The robot
**Environment:** The world
**State:** Current situation
**Action:** What robot can do
**Reward:** Feedback signal
**Policy:** Strategy for choosing actions

### The RL Loop

\`\`\`
State → Agent → Action → Environment → New State + Reward
         ↑__________________________________________|
\`\`\`

---

## 📊 Q-Learning

### Table-Based Q-Learning

\`\`\`python
import numpy as np

class QLearningRobot:
    def __init__(self, n_states, n_actions):
        self.n_states = n_states
        self.n_actions = n_actions
        
        # Q-table: Q(state, action)
        self.q_table = np.zeros((n_states, n_actions))
        
        # Hyperparameters
        self.alpha = 0.1      # Learning rate
        self.gamma = 0.99     # Discount factor
        self.epsilon = 0.1    # Exploration rate
    
    def choose_action(self, state):
        """Epsilon-greedy action selection"""
        if np.random.random() < self.epsilon:
            # Explore: random action
            return np.random.randint(self.n_actions)
        else:
            # Exploit: best known action
            return np.argmax(self.q_table[state])
    
    def update(self, state, action, reward, next_state):
        """Q-learning update rule"""
        # Current Q-value
        current_q = self.q_table[state, action]
        
        # Maximum Q-value for next state
        max_next_q = np.max(self.q_table[next_state])
        
        # Q-learning formula
        new_q = current_q + self.alpha * (
            reward + self.gamma * max_next_q - current_q
        )
        
        self.q_table[state, action] = new_q
    
    def train(self, env, n_episodes=1000):
        """Train the agent"""
        rewards_history = []
        
        for episode in range(n_episodes):
            state = env.reset()
            total_reward = 0
            done = False
            
            while not done:
                # Choose action
                action = self.choose_action(state)
                
                # Take action
                next_state, reward, done = env.step(action)
                
                # Update Q-table
                self.update(state, action, reward, next_state)
                
                state = next_state
                total_reward += reward
            
            rewards_history.append(total_reward)
            
            if episode % 100 == 0:
                avg_reward = np.mean(rewards_history[-100:])
                print(f"Episode {episode}, Avg Reward: {avg_reward:.2f}")
        
        return rewards_history
\`\`\`

---

## 🎯 Deep Q-Network (DQN)

### For Complex State Spaces

\`\`\`python
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random

class DQN(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DQN, self).__init__()
        
        self.network = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim)
        )
    
    def forward(self, x):
        return self.network(x)

class DQNAgent:
    def __init__(self, state_dim, action_dim):
        self.state_dim = state_dim
        self.action_dim = action_dim
        
        # Networks
        self.policy_net = DQN(state_dim, action_dim)
        self.target_net = DQN(state_dim, action_dim)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        
        # Optimizer
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=0.001)
        
        # Replay buffer
        self.memory = deque(maxlen=10000)
        
        # Hyperparameters
        self.gamma = 0.99
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        self.batch_size = 64
    
    def select_action(self, state):
        """Epsilon-greedy action selection"""
        if random.random() < self.epsilon:
            return random.randint(0, self.action_dim - 1)
        
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0)
            q_values = self.policy_net(state_tensor)
            return q_values.argmax().item()
    
    def store_transition(self, state, action, reward, next_state, done):
        """Store experience in replay buffer"""
        self.memory.append((state, action, reward, next_state, done))
    
    def train_step(self):
        """Train on batch from replay buffer"""
        if len(self.memory) < self.batch_size:
            return
        
        # Sample batch
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        
        states = torch.FloatTensor(states)
        actions = torch.LongTensor(actions)
        rewards = torch.FloatTensor(rewards)
        next_states = torch.FloatTensor(next_states)
        dones = torch.FloatTensor(dones)
        
        # Current Q values
        current_q = self.policy_net(states).gather(1, actions.unsqueeze(1))
        
        # Next Q values (using target network)
        with torch.no_grad():
            next_q = self.target_net(next_states).max(1)[0]
            target_q = rewards + (1 - dones) * self.gamma * next_q
        
        # Loss
        loss = nn.MSELoss()(current_q.squeeze(), target_q)
        
        # Optimize
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # Decay epsilon
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
    
    def update_target_network(self):
        """Copy weights from policy to target network"""
        self.target_net.load_state_dict(self.policy_net.state_dict())
\`\`\`

---

## 🎭 Policy Gradient Methods

### REINFORCE Algorithm

\`\`\`python
class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(PolicyNetwork, self).__init__()
        
        self.network = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Softmax(dim=-1)
        )
    
    def forward(self, x):
        return self.network(x)

class REINFORCEAgent:
    def __init__(self, state_dim, action_dim):
        self.policy = PolicyNetwork(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=0.01)
        
        self.gamma = 0.99
        
        # Episode storage
        self.log_probs = []
        self.rewards = []
    
    def select_action(self, state):
        """Sample action from policy"""
        state = torch.FloatTensor(state).unsqueeze(0)
        probs = self.policy(state)
        
        # Sample from distribution
        m = torch.distributions.Categorical(probs)
        action = m.sample()
        
        # Store log probability
        self.log_probs.append(m.log_prob(action))
        
        return action.item()
    
    def store_reward(self, reward):
        """Store reward"""
        self.rewards.append(reward)
    
    def train_episode(self):
        """Train after episode completes"""
        # Calculate returns (discounted rewards)
        returns = []
        R = 0
        for r in reversed(self.rewards):
            R = r + self.gamma * R
            returns.insert(0, R)
        
        returns = torch.tensor(returns)
        
        # Normalize returns
        returns = (returns - returns.mean()) / (returns.std() + 1e-9)
        
        # Calculate loss
        policy_loss = []
        for log_prob, R in zip(self.log_probs, returns):
            policy_loss.append(-log_prob * R)
        
        policy_loss = torch.stack(policy_loss).sum()
        
        # Optimize
        self.optimizer.zero_grad()
        policy_loss.backward()
        self.optimizer.step()
        
        # Clear episode data
        self.log_probs = []
        self.rewards = []
\`\`\`

---

## 🤖 Training in Gazebo

### Gym-like Environment for ROS2

\`\`\`python
import gymnasium as gym
import rclpy

class RobotEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        rclpy.init()
        self.node = Node('robot_env')
        
        # Define action and observation space
        self.action_space = gym.spaces.Discrete(4)  # Forward, back, left, right
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32
        )
        
        # Publishers/Subscribers
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.scan_data = None
    
    def scan_callback(self, msg):
        self.scan_data = np.array(msg.ranges)
    
    def reset(self):
        # Reset robot to starting position
        # ... reset logic ...
        return self.get_observation()
    
    def step(self, action):
        # Execute action
        cmd = Twist()
        if action == 0:  # Forward
            cmd.linear.x = 0.5
        elif action == 1:  # Back
            cmd.linear.x = -0.5
        elif action == 2:  # Left
            cmd.angular.z = 0.5
        elif action == 3:  # Right
            cmd.angular.z = -0.5
        
        self.cmd_pub.publish(cmd)
        
        # Wait for new observation
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        observation = self.get_observation()
        reward = self.calculate_reward()
        done = self.is_done()
        
        return observation, reward, done, {}
    
    def get_observation(self):
        if self.scan_data is not None:
            # Downsample scan data
            return self.scan_data[::36][:10]  # 10 readings
        return np.zeros(10)
    
    def calculate_reward(self):
        # Reward for moving forward without collision
        min_dist = np.min(self.scan_data) if self.scan_data is not None else 10
        
        if min_dist < 0.3:
            return -10  # Collision penalty
        else:
            return 1  # Progress reward
\`\`\`

---

## 📚 Summary

✅ RL learns through trial and error
✅ Q-Learning for discrete actions
✅ DQN for continuous states
✅ Policy Gradients learn stochastic policies
✅ Train in simulation before real robot

**Next:** Chapter 16 - Multi-Robot Systems
`
  },

  {
    id: 16,
    title: "Multi-Robot Systems",
    part: 2,
    partName: "For Researchers",
    startPage: 47,
    endPage: 49,
    difficulty: "Medium",
    estimatedTime: "35 minutes",
    objectives: [
      "Coordinate multiple robots",
      "Implement swarm behaviors",
      "Handle communication and consensus"
    ],
    keywords: ["Multi-Robot", "Swarm", "Coordination", "Consensus", "Formation Control"],
    content: `
# Chapter 16: Multi-Robot Systems

## 🤖🤖🤖 Many Robots, One Goal

**Multi-robot systems** coordinate multiple robots to accomplish complex tasks.

---

## 🎯 Why Multiple Robots?

**Advantages:**
- Parallel task execution
- Redundancy (fault tolerance)
- Distributed sensing
- Scalability

**Challenges:**
- Communication
- Coordination
- Collision avoidance
- Task allocation

---

## 📡 Robot-to-Robot Communication

### Using ROS2 Domain IDs

\`\`\`bash
# Robot 1
export ROS_DOMAIN_ID=1
ros2 run my_package robot_node --ros-args -r __ns:=/robot1

# Robot 2
export ROS_DOMAIN_ID=1
ros2 run my_package robot_node --ros-args -r __ns:=/robot2
\`\`\`

### Message Passing

\`\`\`python
class RobotCommunicator(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}')
        self.robot_id = robot_id
        
        # Publish own state
        self.state_pub = self.create_publisher(
            RobotState, f'/robot_{robot_id}/state', 10
        )
        
        # Subscribe to other robots
        self.other_robots = {}
        for i in range(5):  # 5 robots total
            if i != robot_id:
                self.create_subscription(
                    RobotState,
                    f'/robot_{i}/state',
                    lambda msg, rid=i: self.robot_state_callback(msg, rid),
                    10
                )
    
    def robot_state_callback(self, msg, robot_id):
        """Store state of other robot"""
        self.other_robots[robot_id] = {
            'position': [msg.x, msg.y],
            'velocity': [msg.vx, msg.vy],
            'timestamp': self.get_clock().now()
        }
    
    def broadcast_state(self, x, y, vx, vy):
        """Broadcast own state"""
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.x, msg.y = x, y
        msg.vx, msg.vy = vx, vy
        msg.timestamp = self.get_clock().now().to_msg()
        
        self.state_pub.publish(msg)
\`\`\`

---

## 🐝 Swarm Behaviors

### Flocking (Reynolds' Boids)

Three simple rules create complex behavior:

1. **Separation:** Avoid crowding neighbors
2. **Alignment:** Steer toward average heading
3. **Cohesion:** Steer toward average position

\`\`\`python
class SwarmRobot(Node):
    def __init__(self, robot_id):
        super().__init__(f'swarm_robot_{robot_id}')
        self.robot_id = robot_id
        self.position = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        
        self.neighbors = {}
        
        # Flocking parameters
        self.separation_distance = 1.0
        self.neighbor_distance = 5.0
        
        # Weights
        self.w_separation = 1.5
        self.w_alignment = 1.0
        self.w_cohesion = 1.0
    
    def calculate_separation(self):
        """Avoid crowding neighbors"""
        steer = np.array([0.0, 0.0])
        
        for neighbor in self.neighbors.values():
            diff = self.position - neighbor['position']
            distance = np.linalg.norm(diff)
            
            if distance < self.separation_distance:
                # Repulsion force (inverse distance)
                steer += diff / (distance + 1e-6)
        
        return steer
    
    def calculate_alignment(self):
        """Match velocity with neighbors"""
        if not self.neighbors:
            return np.array([0.0, 0.0])
        
        avg_velocity = np.mean([
            n['velocity'] for n in self.neighbors.values()
        ], axis=0)
        
        return avg_velocity - self.velocity
    
    def calculate_cohesion(self):
        """Move toward center of neighbors"""
        if not self.neighbors:
            return np.array([0.0, 0.0])
        
        center = np.mean([
            n['position'] for n in self.neighbors.values()
        ], axis=0)
        
        return center - self.position
    
    def update_velocity(self):
        """Combine all behaviors"""
        separation = self.calculate_separation()
        alignment = self.calculate_alignment()
        cohesion = self.calculate_cohesion()
        
        # Weighted sum
        steering = (self.w_separation * separation +
                   self.w_alignment * alignment +
                   self.w_cohesion * cohesion)
        
        # Update velocity
        self.velocity += steering * 0.1  # Small step size
        
        # Limit speed
        speed = np.linalg.norm(self.velocity)
        max_speed = 1.0
        if speed > max_speed:
            self.velocity = (self.velocity / speed) * max_speed
        
        return self.velocity
\`\`\`

---

## 📋 Task Allocation

### Market-Based Approach

\`\`\`python
class Task:
    def __init__(self, task_id, position, priority):
        self.id = task_id
        self.position = position
        self.priority = priority
        self.assigned_to = None
        self.bids = {}

class MarketBasedAllocator(Node):
    def __init__(self, robot_id):
        super().__init__(f'allocator_{robot_id}')
        self.robot_id = robot_id
        self.position = np.array([0.0, 0.0])
        
        # Task queue
        self.tasks = []
        
        # Communication
        self.bid_pub = self.create_publisher(Bid, '/bids', 10)
        self.create_subscription(Bid, '/bids', self.bid_callback, 10)
        
        self.current_task = None
    
    def calculate_bid(self, task):
        """Calculate bid for task"""
        # Cost = distance + priority factor
        distance = np.linalg.norm(self.position - task.position)
        cost = distance / (task.priority + 1)
        
        return cost
    
    def auction_task(self, task):
        """Start auction for task"""
        bid_value = self.calculate_bid(task)
        
        # Publish bid
        msg = Bid()
        msg.task_id = task.id
        msg.robot_id = self.robot_id
        msg.bid_value = bid_value
        
        self.bid_pub.publish(msg)
    
    def bid_callback(self, msg):
        """Receive bids from other robots"""
        task = self.get_task_by_id(msg.task_id)
        
        if task:
            task.bids[msg.robot_id] = msg.bid_value
            
            # After timeout, assign to lowest bidder
            if len(task.bids) >= 5:  # All robots bid
                winner = min(task.bids, key=task.bids.get)
                
                if winner == self.robot_id:
                    self.current_task = task
                    self.get_logger().info(f'Won task {task.id}!')
\`\`\`

---

## 🎯 Formation Control

### Leader-Follower

\`\`\`python
class FormationController(Node):
    def __init__(self, robot_id, is_leader=False):
        super().__init__(f'formation_{robot_id}')
        self.robot_id = robot_id
        self.is_leader = is_leader
        
        self.position = np.array([0.0, 0.0])
        self.leader_position = None
        
        # Desired offset from leader
        self.formation_offset = np.array([
            -1.0 * robot_id,  # Stagger behind
            0.0
        ])
    
    def calculate_formation_control(self):
        """Calculate control to maintain formation"""
        if self.is_leader:
            # Leader follows waypoints
            return self.follow_waypoint()
        
        if self.leader_position is None:
            return np.array([0.0, 0.0])
        
        # Desired position in formation
        desired_pos = self.leader_position + self.formation_offset
        
        # Proportional control
        error = desired_pos - self.position
        control = 0.5 * error  # P-gain
        
        return control
\`\`\`

---

## 🗳️ Consensus Algorithms

### Average Consensus

\`\`\`python
class ConsensusNode(Node):
    def __init__(self, robot_id, initial_value):
        super().__init__(f'consensus_{robot_id}')
        self.robot_id = robot_id
        
        # State to reach consensus on
        self.value = initial_value
        
        # Neighbor values
        self.neighbor_values = {}
        
        # Communication
        self.value_pub = self.create_publisher(Float64, f'/value_{robot_id}', 10)
        
        for i in range(5):
            if i != robot_id:
                self.create_subscription(
                    Float64, f'/value_{i}',
                    lambda msg, rid=i: self.update_neighbor(rid, msg.data),
                    10
                )
        
        # Update timer
        self.create_timer(0.1, self.consensus_step)
    
    def update_neighbor(self, robot_id, value):
        """Store neighbor's value"""
        self.neighbor_values[robot_id] = value
    
    def consensus_step(self):
        """Update value toward average"""
        if not self.neighbor_values:
            return
        
        # Calculate average with neighbors
        all_values = [self.value] + list(self.neighbor_values.values())
        avg_value = np.mean(all_values)
        
        # Move toward average
        self.value += 0.1 * (avg_value - self.value)
        
        # Broadcast updated value
        msg = Float64()
        msg.data = self.value
        self.value_pub.publish(msg)
        
        self.get_logger().info(f'Value: {self.value:.2f}')
\`\`\`

---

## 📚 Summary

✅ Multi-robot systems enable complex tasks
✅ Swarm behaviors emerge from simple rules
✅ Market-based task allocation is efficient
✅ Formation control maintains robot geometry
✅ Consensus algorithms synchronize state

**Next:** Part 2 continues with advanced topics!

---

*Part 2 (Chapters 11-16) COMPLETE! Remaining chapters (17-22) coming next!*
`
  },

  {
    id: 17,
    title: "ROS2 Advanced Topics",
    part: 2,
    partName: "For Researchers",
    startPage: 50,
    endPage: 52,
    difficulty: "Medium",
    estimatedTime: "40 minutes",
    objectives: [
      "Master ROS2 lifecycle nodes",
      "Implement custom plugins",
      "Use composition for efficiency"
    ],
    keywords: ["Lifecycle Nodes", "Plugins", "Components", "Composition", "QoS"],
    content: `
# Chapter 17: ROS2 Advanced Topics

## 🔧 Professional ROS2 Techniques

Master advanced ROS2 features for production robotics.

---

## ♻️ Lifecycle Nodes

**Lifecycle nodes** provide controlled state transitions for safety-critical systems.

### States:
- **Unconfigured** → Initial state
- **Inactive** → Configured but not running
- **Active** → Fully operational
- **Finalized** → Shutdown

\`\`\`python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State

class LifecycleRobot(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_robot')
        self.publisher = None
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Called when transitioning to configured state"""
        self.get_logger().info('Configuring...')
        
        # Setup publishers/subscribers
        self.publisher = self.create_lifecycle_publisher(
            String, 'status', 10
        )
        
        # Initialize hardware
        if self.init_hardware():
            return TransitionCallbackReturn.SUCCESS
        return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Called when activating"""
        self.get_logger().info('Activating...')
        
        # Start timers
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Called when deactivating"""
        self.get_logger().info('Deactivating...')
        
        # Stop operations
        if self.timer:
            self.timer.cancel()
        
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Called when cleaning up"""
        self.get_logger().info('Cleaning up...')
        
        # Destroy resources
        self.destroy_lifecycle_publisher(self.publisher)
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Called when shutting down"""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS
    
    def timer_callback(self):
        """Active operation"""
        msg = String()
        msg.data = f'Active at {self.get_clock().now()}'
        self.publisher.publish(msg)

# Transition commands
# ros2 lifecycle set /lifecycle_robot configure
# ros2 lifecycle set /lifecycle_robot activate
# ros2 lifecycle set /lifecycle_robot deactivate
\`\`\`

---

## 🔌 Creating Custom Plugins

### Plugin Interface

\`\`\`cpp
// include/my_package/base_plugin.hpp
#ifndef MY_PACKAGE__BASE_PLUGIN_HPP_
#define MY_PACKAGE__BASE_PLUGIN_HPP_

namespace my_package
{

class BasePlugin
{
public:
  virtual void initialize() = 0;
  virtual double compute(double input) = 0;
  virtual ~BasePlugin() = default;
};

}  // namespace my_package

#endif
\`\`\`

### Plugin Implementation

\`\`\`cpp
// src/simple_plugin.cpp
#include "my_package/base_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_package
{

class SimplePlugin : public BasePlugin
{
public:
  void initialize() override
  {
    // Setup
  }
  
  double compute(double input) override
  {
    return input * 2.0;  // Simple doubling
  }
};

}  // namespace my_package

// Register plugin
PLUGINLIB_EXPORT_CLASS(my_package::SimplePlugin, my_package::BasePlugin)
\`\`\`

### Plugin XML

\`\`\`xml
<!-- plugins.xml -->
<library path="simple_plugin">
  <class type="my_package::SimplePlugin" base_class_type="my_package::BasePlugin">
    <description>A simple plugin that doubles input</description>
  </class>
</library>
\`\`\`

### Using Plugins

\`\`\`cpp
#include "pluginlib/class_loader.hpp"
#include "my_package/base_plugin.hpp"

int main()
{
  pluginlib::ClassLoader<my_package::BasePlugin> loader(
    "my_package", "my_package::BasePlugin"
  );
  
  try {
    auto plugin = loader.createSharedInstance("my_package::SimplePlugin");
    plugin->initialize();
    
    double result = plugin->compute(5.0);
    std::cout << "Result: " << result << std::endl;
  }
  catch(pluginlib::PluginlibException& ex) {
    std::cerr << "Plugin failed: " << ex.what() << std::endl;
  }
  
  return 0;
}
\`\`\`

---

## 🎯 Component Composition

**Components** allow multiple nodes in single process for efficiency.

### Creating a Component

\`\`\`cpp
// src/talker_component.cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

class Talker : public rclcpp::Node
{
public:
  explicit Talker(const rclcpp::NodeOptions & options)
  : Node("talker", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello from component!";
        publisher_->publish(msg);
      }
    );
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition

RCLCPP_COMPONENTS_REGISTER_NODE(composition::Talker)
\`\`\`

### Launching Components

\`\`\`python
# launch/composition.launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Talker',
                name='talker'
            ),
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name='listener'
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([container])
\`\`\`

---

## 📡 Quality of Service (QoS)

**QoS profiles** control message delivery guarantees.

\`\`\`python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class QoSNode(Node):
    def __init__(self):
        super().__init__('qos_node')
        
        # Sensor QoS (best effort, volatile)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Service QoS (reliable, transient local)
        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # System default
        system_qos = QoSProfile(
            reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
            durability=DurabilityPolicy.SYSTEM_DEFAULT
        )
        
        # Create publishers with different QoS
        self.sensor_pub = self.create_publisher(
            LaserScan, '/scan', sensor_qos
        )
        
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', service_qos
        )
\`\`\`

**QoS Policies:**
- **BEST_EFFORT**: Fast, may lose messages
- **RELIABLE**: Slower, guarantees delivery
- **VOLATILE**: Only current data
- **TRANSIENT_LOCAL**: Keeps history for late joiners

---

## 🛠️ Parameter Management

### Dynamic Parameters

\`\`\`python
from rcl_interfaces.msg import SetParametersResult

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with defaults
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('enable_safety', True)
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
    
    def parameter_callback(self, params):
        """Called when parameters change"""
        for param in params:
            if param.name == 'max_speed':
                if param.value > 2.0:
                    self.get_logger().warn('Speed too high!')
                    return SetParametersResult(successful=False)
                
                self.max_speed = param.value
                self.get_logger().info(f'Speed updated: {param.value}')
        
        return SetParametersResult(successful=True)

# Change parameters at runtime:
# ros2 param set /parameter_node max_speed 1.5
\`\`\`

---

## 🔐 Security

### Secure Communication (SROS2)

\`\`\`bash
# Generate keys
ros2 security create_keystore demo_keys
ros2 security create_key demo_keys /my_node

# Run with security
export ROS_SECURITY_KEYSTORE=\${PWD}/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

ros2 run my_package my_node
\`\`\`

---

## 📚 Summary

✅ Lifecycle nodes provide state management
✅ Plugins enable modular architectures
✅ Components improve efficiency
✅ QoS controls message delivery
✅ Dynamic parameters allow runtime config
✅ SROS2 secures communication

**Next:** Chapter 18 - Isaac Sim for Robotics
`
  },

  {
    id: 18,
    title: "Isaac Sim for Robotics",
    part: 2,
    partName: "For Researchers",
    startPage: 53,
    endPage: 55,
    difficulty: "Medium",
    estimatedTime: "45 minutes",
    objectives: [
      "Use NVIDIA Isaac Sim for simulation",
      "Create photorealistic environments",
      "Train AI with synthetic data"
    ],
    keywords: ["Isaac Sim", "Omniverse", "Synthetic Data", "GPU Simulation", "Domain Randomization"],
    content: `
# Chapter 18: Isaac Sim for Robotics

## 🎮 Next-Gen Robot Simulation

**Isaac Sim** = NVIDIA's GPU-accelerated photorealistic simulator built on Omniverse.

---

## 🌟 Why Isaac Sim?

**Advantages over Gazebo:**
- Photorealistic rendering (RTX ray tracing)
- Massive parallelization (1000+ robots)
- Domain randomization built-in
- Synthetic data generation
- Physics on GPU

---

## 🚀 Getting Started

### Installation

\`\`\`bash
# Download Isaac Sim from NVIDIA
# Requires RTX GPU

# Install Isaac Sim ROS2 Bridge
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./python.sh -m pip install rclpy

# Source ROS2
source /opt/ros/humble/setup.bash
\`\`\`

### Launch Isaac Sim

\`\`\`bash
# Start Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# Or headless mode for training
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh --headless
\`\`\`

---

## 🏗️ Building Environments

### USD (Universal Scene Description)

Isaac Sim uses USD format for scenes.

\`\`\`python
from pxr import Usd, UsdGeom, Gf

# Create new stage
stage = Usd.Stage.CreateNew("my_world.usd")

# Add ground plane
xform = UsdGeom.Xform.Define(stage, "/World/Ground")
cube = UsdGeom.Cube.Define(stage, "/World/Ground/Cube")
cube.GetSizeAttr().Set(100.0)
cube.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.5))

# Add light
light = UsdGeom.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(1000)

# Save
stage.Save()
\`\`\`

### Using Isaac Sim Python API

\`\`\`python
from omni.isaac.kit import SimulationApp

# Start simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.robots import Robot
import numpy as np

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add robot
from omni.isaac.wheeled_robots.robots import WheeledRobot
robot = world.scene.add(
    WheeledRobot(
        prim_path="/World/Robot",
        name="my_robot",
        wheel_dof_names=["left_wheel", "right_wheel"],
        create_robot=True,
    )
)

# Add objects
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([2.0, 0, 0.5]),
        size=0.5,
        color=np.array([1, 0, 0])  # Red
    )
)

# Reset world
world.reset()

# Simulation loop
for i in range(1000):
    # Apply robot control
    robot.apply_wheel_actions(
        np.array([0.5, 0.5])  # Both wheels forward
    )
    
    # Step simulation
    world.step(render=True)

simulation_app.close()
\`\`\`

---

## 📷 Synthetic Data Generation

### Camera Setup

\`\`\`python
from omni.isaac.core.utils.camera import Camera

# Create RGB camera
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([3, 0, 2]),
    frequency=30,
    resolution=(640, 480)
)

# Enable depth and segmentation
camera.initialize()
camera.add_depth_to_frame()
camera.add_semantic_segmentation_to_frame()

# Capture frame
rgb = camera.get_rgba()[:, :, :3]
depth = camera.get_depth()
segmentation = camera.get_semantic_segmentation()
\`\`\`

### Domain Randomization

\`\`\`python
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import random

class DomainRandomizer:
    def __init__(self, world):
        self.world = world
    
    def randomize_lighting(self):
        """Randomize light intensity and color"""
        intensity = random.uniform(500, 2000)
        # Set light intensity
        
    def randomize_object_pose(self, object_name):
        """Randomize object position and orientation"""
        obj = self.world.scene.get_object(object_name)
        
        # Random position
        x = random.uniform(-2, 2)
        y = random.uniform(-2, 2)
        z = random.uniform(0.5, 1.5)
        
        # Random orientation
        roll = random.uniform(0, 2*np.pi)
        pitch = random.uniform(0, 2*np.pi)
        yaw = random.uniform(0, 2*np.pi)
        
        quat = euler_angles_to_quat([roll, pitch, yaw])
        
        obj.set_world_pose(
            position=np.array([x, y, z]),
            orientation=quat
        )
    
    def randomize_textures(self):
        """Randomize material properties"""
        # Change colors, textures, reflectivity
        pass
    
    def randomize_all(self):
        """Full randomization"""
        self.randomize_lighting()
        for obj in ["cube", "sphere", "cylinder"]:
            self.randomize_object_pose(obj)
        self.randomize_textures()

# Usage in training loop
randomizer = DomainRandomizer(world)

for episode in range(1000):
    world.reset()
    randomizer.randomize_all()
    
    # Collect data
    for step in range(100):
        rgb = camera.get_rgba()
        # Train model
        world.step()
\`\`\`

---

## 🤖 ROS2 Integration

### Isaac Sim ROS2 Bridge

\`\`\`python
import rclpy
from omni.isaac.core_nodes.scripts.utils import set_target_prims

# Enable ROS2 bridge
import omni.isaac.ros2_bridge

# Create ROS2 publishers
from std_msgs.msg import String

# Publish camera data
camera_pub = self.node.create_publisher(
    Image, '/camera/image_raw', 10
)

# Subscribe to velocity commands
def cmd_vel_callback(msg):
    robot.apply_wheel_actions(
        np.array([msg.linear.x, msg.angular.z])
    )

cmd_sub = self.node.create_subscription(
    Twist, '/cmd_vel', cmd_vel_callback, 10
)
\`\`\`

---

## 🏋️ Training with Isaac Sim

### Reinforcement Learning Setup

\`\`\`python
from omni.isaac.gym.vec_env import VecEnvBase
import torch

class RobotEnv(VecEnvBase):
    def __init__(self, num_envs=256):
        super().__init__(num_envs=num_envs)
        
        # Create parallel environments on GPU
        self.num_envs = num_envs
        
    def reset(self):
        """Reset all environments"""
        # Parallel reset on GPU
        observations = torch.zeros((self.num_envs, 10))
        return observations
    
    def step(self, actions):
        """Step all environments in parallel"""
        # Apply actions
        # Run physics
        # Calculate rewards
        
        observations = torch.zeros((self.num_envs, 10))
        rewards = torch.zeros(self.num_envs)
        dones = torch.zeros(self.num_envs, dtype=torch.bool)
        
        return observations, rewards, dones

# Train with 256 parallel environments!
env = RobotEnv(num_envs=256)
# ... RL training code ...
\`\`\`

---

## 🎯 Real-to-Sim Transfer

### Calibrating Physics

\`\`\`python
# Match real-world physics
from omni.isaac.core.physics_context import PhysicsContext

physics_ctx = PhysicsContext()
physics_ctx.set_gravity(-9.81)

# Set material properties
cube.get_applied_visual_material().set_roughness(0.5)
cube.get_applied_physics_material().set_static_friction(0.7)
cube.get_applied_physics_material().set_dynamic_friction(0.5)
\`\`\`

---

## 📚 Summary

✅ Isaac Sim provides photorealistic simulation
✅ GPU acceleration enables massive parallelization
✅ Domain randomization creates robust policies
✅ Synthetic data generation for vision
✅ ROS2 bridge for seamless integration
✅ Perfect for deep learning training

**Next:** Chapter 19 - Human-Robot Interaction
`
  },

  {
    id: 19,
    title: "Human-Robot Interaction",
    part: 2,
    partName: "For Researchers",
    startPage: 56,
    endPage: 58,
    difficulty: "Medium",
    estimatedTime: "35 minutes",
    objectives: [
      "Design intuitive robot interfaces",
      "Implement gesture recognition",
      "Handle voice commands and dialog"
    ],
    keywords: ["HRI", "Gesture Recognition", "Voice Control", "Natural Language", "Social Robots"],
    content: `
# Chapter 19: Human-Robot Interaction

## 🤝 Making Robots User-Friendly

**HRI** = Human-Robot Interaction - designing robots that people can work with naturally.

---

## 🗣️ Voice Control

### Speech Recognition with Vosk

\`\`\`python
from vosk import Model, KaldiRecognizer
import pyaudio
import json

class VoiceController(Node):
    def __init__(self):
        super().__init__('voice_controller')
        
        # Load speech model
        self.model = Model("model")
        self.recognizer = KaldiRecognizer(self.model, 16000)
        
        # Audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8000
        )
        
        # Command publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Start listening
        self.create_timer(0.1, self.listen)
    
    def listen(self):
        """Listen for voice commands"""
        data = self.stream.read(4000, exception_on_overflow=False)
        
        if self.recognizer.AcceptWaveform(data):
            result = json.loads(self.recognizer.Result())
            text = result.get('text', '')
            
            if text:
                self.process_command(text)
    
    def process_command(self, text):
        """Process voice command"""
        cmd = Twist()
        
        if 'forward' in text or 'go' in text:
            cmd.linear.x = 0.5
            self.get_logger().info('Moving forward')
        
        elif 'back' in text or 'reverse' in text:
            cmd.linear.x = -0.5
            self.get_logger().info('Moving backward')
        
        elif 'left' in text or 'turn left' in text:
            cmd.angular.z = 0.5
            self.get_logger().info('Turning left')
        
        elif 'right' in text or 'turn right' in text:
            cmd.angular.z = -0.5
            self.get_logger().info('Turning right')
        
        elif 'stop' in text or 'halt' in text:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Stopping')
        
        self.cmd_pub.publish(cmd)
\`\`\`

### Text-to-Speech Response

\`\`\`python
import pyttsx3

class RobotSpeaker(Node):
    def __init__(self):
        super().__init__('robot_speaker')
        
        # Initialize TTS engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)  # Speed
        self.engine.setProperty('volume', 0.9)
        
        # Subscribe to status updates
        self.create_subscription(
            String, '/robot/status', 
            self.status_callback, 10
        )
    
    def speak(self, text):
        """Speak text"""
        self.get_logger().info(f'Speaking: {text}')
        self.engine.say(text)
        self.engine.runAndWait()
    
    def status_callback(self, msg):
        """Announce status changes"""
        self.speak(msg.data)

# Usage
speaker = RobotSpeaker()
speaker.speak("Hello! I am ready to assist you.")
\`\`\`

---

## 👋 Gesture Recognition

### MediaPipe Hand Tracking

\`\`\`python
import mediapipe as mp
import cv2

class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        
        # Initialize MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Camera subscription
        self.bridge = CvBridge()
        self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10
        )
        
        # Command publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def image_callback(self, msg):
        """Process camera image for gestures"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Detect hands
        results = self.hands.process(rgb_image)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks
                self.mp_draw.draw_landmarks(
                    cv_image, hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS
                )
                
                # Recognize gesture
                gesture = self.recognize_gesture(hand_landmarks)
                if gesture:
                    self.execute_gesture(gesture)
        
        cv2.imshow('Gesture Control', cv_image)
        cv2.waitKey(1)
    
    def recognize_gesture(self, landmarks):
        """Recognize hand gesture"""
        # Get key points
        thumb_tip = landmarks.landmark[4]
        index_tip = landmarks.landmark[8]
        middle_tip = landmarks.landmark[12]
        ring_tip = landmarks.landmark[16]
        pinky_tip = landmarks.landmark[20]
        
        palm = landmarks.landmark[0]
        
        # Thumbs up
        if thumb_tip.y < palm.y and all([
            index_tip.y > palm.y,
            middle_tip.y > palm.y,
            ring_tip.y > palm.y,
            pinky_tip.y > palm.y
        ]):
            return 'thumbs_up'
        
        # Open palm (stop)
        if all([
            thumb_tip.y < palm.y,
            index_tip.y < palm.y,
            middle_tip.y < palm.y,
            ring_tip.y < palm.y,
            pinky_tip.y < palm.y
        ]):
            return 'open_palm'
        
        # Pointing (direction)
        if index_tip.y < palm.y and all([
            middle_tip.y > palm.y,
            ring_tip.y > palm.y,
            pinky_tip.y > palm.y
        ]):
            # Determine direction
            if index_tip.x < palm.x - 0.1:
                return 'point_left'
            elif index_tip.x > palm.x + 0.1:
                return 'point_right'
        
        return None
    
    def execute_gesture(self, gesture):
        """Execute robot command based on gesture"""
        cmd = Twist()
        
        if gesture == 'thumbs_up':
            cmd.linear.x = 0.5
            self.get_logger().info('Gesture: Move forward')
        
        elif gesture == 'open_palm':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Gesture: Stop')
        
        elif gesture == 'point_left':
            cmd.angular.z = 0.5
            self.get_logger().info('Gesture: Turn left')
        
        elif gesture == 'point_right':
            cmd.angular.z = -0.5
            self.get_logger().info('Gesture: Turn right')
        
        self.cmd_pub.publish(cmd)
\`\`\`

---

## 💬 Natural Language Dialog

### ChatGPT Integration

\`\`\`python
import openai

class RobotDialog(Node):
    def __init__(self):
        super().__init__('robot_dialog')
        
        # OpenAI API
        openai.api_key = "your-api-key"
        
        # Conversation history
        self.messages = [
            {"role": "system", "content": "You are a helpful robot assistant."}
        ]
        
        # Speech components
        self.speaker = RobotSpeaker()
    
    def chat(self, user_input):
        """Have conversation with user"""
        # Add user message
        self.messages.append({
            "role": "user",
            "content": user_input
        })
        
        # Get AI response
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=self.messages,
            max_tokens=150
        )
        
        reply = response.choices[0].message.content
        
        # Add to history
        self.messages.append({
            "role": "assistant",
            "content": reply
        })
        
        # Speak response
        self.speaker.speak(reply)
        
        return reply

# Example conversation
dialog = RobotDialog()
dialog.chat("Can you help me find the kitchen?")
dialog.chat("What time is it?")
\`\`\`

---

## 🤖 Social Robot Behaviors

### Emotional Expressions

\`\`\`python
class SocialBehaviors(Node):
    def __init__(self):
        super().__init__('social_behaviors')
        
        # LED publisher (for face display)
        self.led_pub = self.create_publisher(
            LEDPattern, '/led/pattern', 10
        )
        
        # Sound publisher
        self.sound_pub = self.create_publisher(
            String, '/sound/play', 10
        )
    
    def express_emotion(self, emotion):
        """Display emotion"""
        if emotion == 'happy':
            self.show_led_pattern('smile')
            self.play_sound('happy_beep.wav')
        
        elif emotion == 'sad':
            self.show_led_pattern('frown')
            self.play_sound('sad_beep.wav')
        
        elif emotion == 'thinking':
            self.show_led_pattern('dots_moving')
        
        elif emotion == 'surprised':
            self.show_led_pattern('wide_eyes')
            self.play_sound('surprise.wav')
    
    def greet_person(self):
        """Greet detected person"""
        self.express_emotion('happy')
        self.speaker.speak("Hello! How can I help you today?")
    
    def acknowledge_command(self):
        """Acknowledge received command"""
        self.express_emotion('thinking')
        self.speaker.speak("Understood. Working on it.")
    
    def task_complete(self):
        """Indicate task completion"""
        self.express_emotion('happy')
        self.speaker.speak("Task completed successfully!")
\`\`\`

---

## 📊 Measuring HRI Quality

### Usability Metrics

\`\`\`python
class HRIMetrics(Node):
    def __init__(self):
        super().__init__('hri_metrics')
        
        # Track interactions
        self.command_success = 0
        self.command_failure = 0
        self.response_times = []
        
        self.start_time = self.get_clock().now()
    
    def log_command(self, success, response_time):
        """Log command outcome"""
        if success:
            self.command_success += 1
        else:
            self.command_failure += 1
        
        self.response_times.append(response_time)
    
    def get_metrics(self):
        """Calculate HRI metrics"""
        total = self.command_success + self.command_failure
        success_rate = self.command_success / total if total > 0 else 0
        avg_response = np.mean(self.response_times) if self.response_times else 0
        
        return {
            'success_rate': success_rate,
            'avg_response_time': avg_response,
            'total_interactions': total
        }
\`\`\`

---

## 📚 Summary

✅ Voice control enables hands-free operation
✅ Gesture recognition provides intuitive input
✅ Natural language dialog makes robots approachable
✅ Social behaviors improve user experience
✅ Metrics help evaluate HRI quality

**Next:** Chapter 20 - Safety and Ethics
`
  },

  {
    id: 20,
    title: "Safety and Ethics in Robotics",
    part: 2,
    partName: "For Researchers",
    startPage: 59,
    endPage: 60,
    difficulty: "Medium",
    estimatedTime: "30 minutes",
    objectives: [
      "Implement robot safety systems",
      "Understand ethical considerations",
      "Design fail-safe mechanisms"
    ],
    keywords: ["Safety", "Ethics", "Fail-safe", "E-stop", "Risk Assessment", "ISO Standards"],
    content: `
# Chapter 20: Safety and Ethics in Robotics

## ⚠️ Building Responsible Robots

Safety and ethics must be core principles, not afterthoughts!

---

## 🛡️ Safety Systems

### Emergency Stop (E-Stop)

\`\`\`python
class SafetySystem(Node):
    def __init__(self):
        super().__init__('safety_system')
        
        # Emergency stop flag
        self.emergency_stop = False
        
        # Subscribe to E-stop button
        self.create_subscription(
            Bool, '/emergency_stop', 
            self.estop_callback, 10
        )
        
        # Monitor robot state
        self.create_timer(0.1, self.safety_check)
        
        # Control publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def estop_callback(self, msg):
        """Handle emergency stop"""
        if msg.data:
            self.emergency_stop = True
            self.execute_emergency_stop()
            self.get_logger().error('EMERGENCY STOP ACTIVATED!')
    
    def execute_emergency_stop(self):
        """Immediately stop all motion"""
        # Stop motors
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        
        # Disable actuators
        self.disable_all_motors()
        
        # Alert operators
        self.publish_alarm()
    
    def safety_check(self):
        """Continuous safety monitoring"""
        if self.emergency_stop:
            return  # Don't allow any action
        
        # Check sensor failures
        if self.is_sensor_failed():
            self.execute_emergency_stop()
        
        # Check collision imminent
        if self.is_collision_imminent():
            self.execute_collision_avoidance()
        
        # Check system health
        if not self.is_system_healthy():
            self.enter_safe_mode()
\`\`\`

### Collision Avoidance

\`\`\`python
class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')
        
        # Safety zones
        self.DANGER_ZONE = 0.3  # meters
        self.WARNING_ZONE = 0.8  # meters
        
        self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, 10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
    
    def scan_callback(self, msg):
        """Check for obstacles"""
        min_distance = min(msg.ranges)
        
        if min_distance < self.DANGER_ZONE:
            # STOP immediately
            self.publish_stop()
            self.get_logger().warn(f'Obstacle at {min_distance}m - STOPPING')
        
        elif min_distance < self.WARNING_ZONE:
            # Slow down
            self.reduce_speed(factor=0.5)
            self.get_logger().info(f'Obstacle at {min_distance}m - SLOWING')
    
    def filter_command(self, desired_cmd):
        """Filter unsafe commands"""
        # Check if command would cause collision
        if self.would_collide(desired_cmd):
            return Twist()  # Return stop command
        
        # Limit speed near obstacles
        safe_cmd = self.limit_speed(desired_cmd)
        
        return safe_cmd
\`\`\`

---

## 📋 Risk Assessment

### HAZOP Analysis

\`\`\`python
class RiskAssessment:
    """Hazard and Operability Study"""
    
    def __init__(self):
        self.hazards = []
    
    def analyze_hazard(self, component, failure_mode):
        """Analyze potential hazard"""
        hazard = {
            'component': component,
            'failure_mode': failure_mode,
            'likelihood': self.assess_likelihood(failure_mode),
            'severity': self.assess_severity(failure_mode),
            'mitigation': self.suggest_mitigation(failure_mode)
        }
        
        hazard['risk_level'] = (
            hazard['likelihood'] * hazard['severity']
        )
        
        self.hazards.append(hazard)
        return hazard
    
    def assess_likelihood(self, failure_mode):
        """Rate likelihood (1-5)"""
        # 1 = Rare, 5 = Frequent
        likelihood_map = {
            'sensor_failure': 2,
            'motor_failure': 2,
            'software_bug': 3,
            'communication_loss': 3,
            'power_failure': 1
        }
        return likelihood_map.get(failure_mode, 3)
    
    def assess_severity(self, failure_mode):
        """Rate severity (1-5)"""
        # 1 = Minor, 5 = Catastrophic
        severity_map = {
            'sensor_failure': 4,
            'motor_failure': 4,
            'software_bug': 3,
            'communication_loss': 4,
            'power_failure': 5
        }
        return severity_map.get(failure_mode, 3)
    
    def suggest_mitigation(self, failure_mode):
        """Suggest risk mitigation"""
        mitigations = {
            'sensor_failure': 'Redundant sensors, watchdog timer',
            'motor_failure': 'Current monitoring, thermal protection',
            'software_bug': 'Extensive testing, formal verification',
            'communication_loss': 'Timeout detection, fail-safe mode',
            'power_failure': 'Battery backup, graceful shutdown'
        }
        return mitigations.get(failure_mode, 'Unknown')
    
    def generate_report(self):
        """Generate safety report"""
        high_risk = [h for h in self.hazards if h['risk_level'] >= 12]
        
        report = "SAFETY RISK ASSESSMENT\\n"
        report += "=" * 50 + "\\n\\n"
        
        if high_risk:
            report += "HIGH RISK HAZARDS:\\n"
            for hazard in high_risk:
                report += f"- {hazard['component']}: {hazard['failure_mode']}\\n"
                report += f"  Risk Level: {hazard['risk_level']}\\n"
                report += f"  Mitigation: {hazard['mitigation']}\\n\\n"
        
        return report

# Usage
assessment = RiskAssessment()
assessment.analyze_hazard('LiDAR', 'sensor_failure')
assessment.analyze_hazard('Motor Driver', 'motor_failure')
print(assessment.generate_report())
\`\`\`

---

## 🤔 Ethical Considerations

### Ethical Decision Framework

\`\`\`python
class EthicalDecisionMaker:
    """Framework for ethical robot decisions"""
    
    def __init__(self):
        # Asimov's Laws as baseline
        self.laws = [
            "Do not harm humans",
            "Obey humans (unless conflicts with law 1)",
            "Protect own existence (unless conflicts with laws 1-2)"
        ]
    
    def evaluate_action(self, action, context):
        """Evaluate if action is ethical"""
        scores = {
            'harm_to_humans': 0,
            'benefit_to_humans': 0,
            'legal_compliance': 0,
            'transparency': 0
        }
        
        # Analyze potential harm
        if self.causes_harm(action, context):
            scores['harm_to_humans'] = -10
            return False, "Action would cause harm"
        
        # Analyze benefit
        scores['benefit_to_humans'] = self.calculate_benefit(action)
        
        # Check legal compliance
        if not self.is_legal(action):
            scores['legal_compliance'] = -5
            return False, "Action not legally compliant"
        
        # Transparency check
        if self.is_explainable(action):
            scores['transparency'] = 5
        
        total_score = sum(scores.values())
        
        return total_score > 0, scores
    
    def causes_harm(self, action, context):
        """Check if action could harm humans"""
        # Check collision with people
        if action['type'] == 'move':
            if self.people_in_path(context):
                return True
        
        # Check privacy violations
        if action['type'] == 'record':
            if not self.has_consent(context):
                return True
        
        return False
    
    def calculate_benefit(self, action):
        """Estimate benefit of action"""
        benefits = {
            'assist_human': 10,
            'deliver_item': 5,
            'provide_information': 3,
            'idle': 0
        }
        return benefits.get(action['type'], 0)
\`\`\`

---

## 📜 Standards and Regulations

### ISO 13482 (Service Robots)

**Key Requirements:**
- Risk assessment and reduction
- Protective measures
- User information
- Verification and validation

### Implementation Checklist

\`\`\`python
class ISO13482Compliance:
    def __init__(self):
        self.checklist = {
            'risk_assessment_complete': False,
            'emergency_stop_functional': False,
            'safety_distances_verified': False,
            'collision_detection_active': False,
            'documentation_complete': False,
            'user_training_provided': False
        }
    
    def verify_compliance(self):
        """Check compliance status"""
        total = len(self.checklist)
        completed = sum(self.checklist.values())
        
        compliance_rate = (completed / total) * 100
        
        if compliance_rate == 100:
            return "FULLY COMPLIANT"
        elif compliance_rate >= 80:
            return "MOSTLY COMPLIANT - Address remaining items"
        else:
            return "NON-COMPLIANT - Significant work needed"
\`\`\`

---

## 🔒 Data Privacy

### GDPR Compliance for Robots

\`\`\`python
class PrivacyManager(Node):
    def __init__(self):
        super().__init__('privacy_manager')
        
        self.consent_given = {}
        self.data_retention_days = 30
    
    def request_consent(self, person_id, data_type):
        """Request consent before collecting data"""
        # Display consent request
        self.display_message(
            f"We would like to collect {data_type} data. "
            "Do you consent?"
        )
        
        # Wait for response
        consent = self.wait_for_user_response()
        
        self.consent_given[person_id] = {
            'data_type': data_type,
            'consent': consent,
            'timestamp': self.get_clock().now()
        }
        
        return consent
    
    def anonymize_data(self, data):
        """Remove personally identifiable information"""
        # Remove faces from images
        # Hash IDs
        # Remove location data
        return anonymized_data
    
    def enforce_retention_policy(self):
        """Delete old data"""
        cutoff_date = self.get_clock().now() - Duration(days=self.data_retention_days)
        
        # Delete data older than retention period
        self.delete_data_before(cutoff_date)
\`\`\`

---

## 📚 Summary

✅ Emergency stops are mandatory
✅ Collision avoidance prevents harm
✅ Risk assessment identifies hazards
✅ Ethical frameworks guide decisions
✅ Standards compliance ensures safety
✅ Privacy protection is essential

**Next:** Chapter 21 - Publishing Your Research
`
  },

  {
    id: 21,
    title: "Publishing Your Research",
    part: 2,
    partName: "For Researchers",
    startPage: 61,
    endPage: 61,
    difficulty: "Medium",
    estimatedTime: "25 minutes",
    objectives: [
      "Write effective research papers",
      "Present work at conferences",
      "Share code and datasets"
    ],
    keywords: ["Research Paper", "Conference", "Open Source", "Reproducibility", "Academic Writing"],
    content: `
# Chapter 21: Publishing Your Research

## 📄 Sharing Your Work with the World

Research isn't complete until it's published and reproducible!

---

## ✍️ Writing Research Papers

### Paper Structure

**Standard Format:**
1. **Abstract** (150-250 words)
2. **Introduction** - Problem + motivation
3. **Related Work** - Literature review
4. **Method** - Your approach
5. **Experiments** - Results
6. **Discussion** - Analysis
7. **Conclusion** - Summary + future work

### Abstract Template

\`\`\`
[Context] Physical AI robots require efficient navigation in dynamic environments.

[Problem] Existing methods struggle with real-time obstacle avoidance while maintaining smooth trajectories.

[Solution] We propose DynamicPath, a novel planning algorithm that combines...

[Results] Experiments show 35% faster computation and 20% smoother paths compared to state-of-the-art.

[Impact] This enables deployment in crowded human environments.
\`\`\`

### Method Section Guidelines

**Be specific:**
- ❌ "We used a neural network"
- ✅ "We used a 5-layer CNN with ReLU activations, batch normalization, and dropout (p=0.5)"

**Include pseudocode:**
\`\`\`
Algorithm 1: DynamicPath Planning
Input: current_pose, goal, obstacles
Output: trajectory

1: Initialize path P = []
2: while distance(current, goal) > threshold do
3:     candidates = sample_velocities()
4:     for each v in candidates do
5:         score[v] = evaluate(v, obstacles, goal)
6:     best_v = argmax(score)
7:     P.append(best_v)
8:     current = simulate_step(current, best_v)
9: return P
\`\`\`

---

## 📊 Presenting Results

### Creating Effective Figures

\`\`\`python
import matplotlib.pyplot as plt
import numpy as np

# Publication-quality plot
plt.figure(figsize=(6, 4), dpi=300)

# Data
methods = ['Baseline', 'Method A', 'Ours']
success_rate = [75, 82, 91]
std = [5, 4, 3]

# Bar plot with error bars
plt.bar(methods, success_rate, yerr=std, capsize=5,
       color=['#E74C3C', '#3498DB', '#2ECC71'])

plt.ylabel('Success Rate (%)', fontsize=12)
plt.title('Navigation Success Rate', fontsize=14, fontweight='bold')
plt.ylim([0, 100])
plt.grid(axis='y', alpha=0.3)

# Save for paper
plt.tight_layout()
plt.savefig('results_success_rate.pdf', format='pdf', bbox_inches='tight')
plt.show()
\`\`\`

### Comparison Table

\`\`\`latex
\\begin{table}[ht]
\\centering
\\caption{Comparison of Path Planning Methods}
\\begin{tabular}{lcccc}
\\hline
Method & Success (\\%) & Time (ms) & Smoothness & Memory (MB) \\\\
\\hline
A* & 78.3 & 45.2 & 0.65 & 12.4 \\\\
RRT & 82.1 & 38.7 & 0.58 & 15.2 \\\\
DWA & 85.4 & 12.3 & 0.72 & 8.7 \\\\
\\textbf{Ours} & \\textbf{91.2} & \\textbf{15.8} & \\textbf{0.89} & \\textbf{10.1} \\\\
\\hline
\\end{tabular}
\\label{tab:comparison}
\\end{table}
\`\`\`

---

## 🎤 Conference Presentations

### Creating Slides

**Slide Structure (10-minute talk):**
1. Title (1 slide)
2. Motivation/Problem (2 slides)
3. Related Work (1 slide)
4. Our Method (3-4 slides)
5. Experiments (2-3 slides)
6. Conclusion (1 slide)

**Design Tips:**
- Use large fonts (≥24pt)
- Minimal text (6 lines max per slide)
- High-quality images
- Consistent color scheme
- Animations sparingly

### Example Slide (LaTeX Beamer)

\`\`\`latex
\\begin{frame}{Our Approach}
\\begin{itemize}
    \\item \\textbf{Key Idea:} Combine global and local planning
    \\begin{itemize}
        \\item Global: A* on coarse grid
        \\item Local: DWA for dynamic obstacles
    \\end{itemize}
    \\item \\textbf{Advantage:} Best of both worlds
    \\begin{itemize}
        \\item Optimal paths from A*
        \\item Real-time reactivity from DWA
    \\end{itemize}
\\end{itemize}

\\begin{center}
\\includegraphics[width=0.7\\textwidth]{method_diagram.pdf}
\\end{center}
\\end{frame}
\`\`\`

---

## 💻 Open Source Your Code

### Repository Structure

\`\`\`
my-research-project/
├── README.md              ← Start here
├── requirements.txt       ← Dependencies
├── setup.py              ← Installation
├── LICENSE               ← MIT/Apache/GPL
├── docs/
│   ├── installation.md
│   └── usage.md
├── src/
│   ├── __init__.py
│   ├── planner.py
│   └── simulator.py
├── experiments/
│   ├── run_baseline.py
│   └── run_ours.py
├── data/
│   └── README.md
└── results/
    └── figures/
\`\`\`

### README Template

\`\`\`markdown
# DynamicPath: Real-time Navigation for Mobile Robots

[![Paper](https://img.shields.io/badge/paper-arxiv-red)](link)
[![License](https://img.shields.io/badge/license-MIT-blue)]()

Official implementation of "DynamicPath: Efficient Navigation in Dynamic Environments" (ICRA 2024)

## Installation

\`\`\`bash
git clone https://github.com/yourname/dynamicpath
cd dynamicpath
pip install -r requirements.txt
\`\`\`

## Quick Start

\`\`\`python
from dynamicpath import Planner

planner = Planner()
trajectory = planner.plan(start, goal, obstacles)
\`\`\`

## Reproduce Results

\`\`\`bash
python experiments/run_all.py
\`\`\`

## Citation

If you use this code, please cite:

\`\`\`bibtex
@inproceedings{yourname2024dynamicpath,
  title={DynamicPath: Efficient Navigation in Dynamic Environments},
  author={Your Name},
  booktitle={ICRA},
  year={2024}
}
\`\`\`

## License

MIT License
\`\`\`

---

## 📦 Sharing Datasets

### Dataset README

\`\`\`markdown
# Robot Navigation Dataset

## Overview
- 10,000 navigation episodes
- 5 different environments
- RGB-D images + LiDAR scans
- Ground truth trajectories

## Download
[Link to dataset] (5.2 GB)

## Format

\`\`\`
dataset/
├── env1/
│   ├── episode_0001/
│   │   ├── rgb/
│   │   ├── depth/
│   │   ├── scan.txt
│   │   └── trajectory.csv
│   └── ...
└── ...
\`\`\`

## Citation
[bibtex]
\`\`\`

---

## 🏆 Top Robotics Conferences

**Tier 1:**
- ICRA (International Conference on Robotics and Automation)
- IROS (Intelligent Robots and Systems)
- RSS (Robotics: Science and Systems)
- CoRL (Conference on Robot Learning)

**Tier 2:**
- ICARCV, Humanoids, ISER

**Journals:**
- IEEE TRO (Transactions on Robotics)
- IJRR (International Journal of Robotics Research)
- Robotics and Autonomous Systems

---

## 📝 Review Process Tips

**Responding to Reviews:**
- Thank reviewers
- Address every comment
- Be respectful even if review is harsh
- Highlight changes in revised manuscript

**Common Rejection Reasons:**
- Insufficient novelty
- Poor experimental validation
- Missing comparisons with baselines
- Unclear writing
- Not reproducible

---

## 📚 Summary

✅ Follow standard paper structure
✅ Create publication-quality figures
✅ Practice your presentation
✅ Open source your code
✅ Make work reproducible
✅ Respond professionally to reviews

---

**🎉 PART 2 COMPLETE!**

You've mastered:
- Advanced simulation
- Computer vision
- SLAM
- Motion planning
- Reinforcement learning
- Multi-robot systems
- HRI
- Safety & ethics
- Research methodology

**Ready for Part 3: For Experts?** → Chapter 22!
`
  },

  {
    id: 22,
    title: "Part 2 Summary and Projects",
    part: 2,
    partName: "For Researchers",
    startPage: 62,
    endPage: 62,
    difficulty: "Medium",
    estimatedTime: "20 minutes",
    objectives: [
      "Review Part 2 concepts",
      "Explore research project ideas",
      "Prepare for expert-level topics"
    ],
    keywords: ["Summary", "Projects", "Research Ideas", "Advanced Topics"],
    content: `
# Chapter 22: Part 2 Summary and Projects

## 🎓 You've Mastered Research-Level Robotics!

Congratulations on completing Part 2: For Researchers!

---

## 📚 What You've Learned

### Advanced Tools & Frameworks
✅ **Gazebo** - Professional simulation
✅ **Isaac Sim** - GPU-accelerated photorealistic simulation
✅ **ROS2 Advanced** - Lifecycle nodes, plugins, components
✅ **Computer Vision** - YOLO, tracking, 3D perception

### Algorithms & Methods
✅ **SLAM** - Cartographer, SLAM Toolbox, localization
✅ **Motion Planning** - RRT, A*, DWA, optimization
✅ **Reinforcement Learning** - Q-learning, DQN, policy gradients
✅ **Multi-Robot Systems** - Swarm, coordination, consensus

### Professional Skills
✅ **Human-Robot Interaction** - Voice, gestures, natural language
✅ **Safety & Ethics** - Risk assessment, standards, privacy
✅ **Research Methodology** - Writing papers, presenting, open source

---

## 🚀 Research Project Ideas

### Project 1: Autonomous Warehouse Robot

**Goal:** Robot navigates warehouse, picks items, delivers to packing station

**Technologies:**
- Nav2 for navigation
- MoveIt2 for manipulation
- YOLO for object detection
- RL for task optimization

**Challenges:**
- Dynamic obstacles (humans, forklifts)
- Multi-robot coordination
- Real-time decision making

**Expected Outcome:** Published paper at ICRA/IROS

---

### Project 2: Social Companion Robot

**Goal:** Robot assists elderly people with daily tasks

**Technologies:**
- MediaPipe for gesture recognition
- Speech recognition + TTS
- Sentiment analysis
- Task planning

**Challenges:**
- Natural human interaction
- Understanding context
- Safety around vulnerable users
- Privacy concerns

**Expected Outcome:** HRI conference paper

---

### Project 3: Agricultural Inspection Robot

**Goal:** Autonomous crop monitoring and disease detection

**Technologies:**
- Visual SLAM for localization
- Deep learning for plant health
- Path planning for coverage
- Multi-spectral imaging

**Challenges:**
- Outdoor navigation (GPS-denied)
- Varying lighting conditions
- Unstructured environment
- Long battery life

**Expected Outcome:** Field Robotics paper

---

### Project 4: Multi-Robot Search and Rescue

**Goal:** Team of robots search disaster area for survivors

**Technologies:**
- Multi-robot SLAM
- Swarm coordination
- Thermal imaging
- Mesh networking

**Challenges:**
- Communication in degraded environment
- Unknown/changing terrain
- Resource allocation
- Human detection accuracy

**Expected Outcome:** IROS/RSS paper

---

### Project 5: Reinforcement Learning for Manipulation

**Goal:** Train robot to grasp novel objects using RL

**Technologies:**
- Isaac Sim for training
- PPO/SAC algorithms
- Domain randomization
- Sim-to-real transfer

**Challenges:**
- Sample efficiency
- Generalization to real world
- Contact dynamics
- Multi-step tasks

**Expected Outcome:** CoRL paper

---

## 📊 Skills Self-Assessment

Rate yourself (1-5) on each topic:

**Simulation:**
- [ ] Gazebo world building
- [ ] Isaac Sim environments
- [ ] Physics tuning
- [ ] Sensor simulation

**Perception:**
- [ ] Object detection (YOLO)
- [ ] 3D vision (point clouds)
- [ ] SLAM implementation
- [ ] Visual servoing

**Planning:**
- [ ] Sampling-based planners
- [ ] Graph search algorithms
- [ ] Trajectory optimization
- [ ] Multi-robot coordination

**Learning:**
- [ ] Q-learning/DQN
- [ ] Policy gradients
- [ ] Reward design
- [ ] Training pipelines

**Professional:**
- [ ] Writing papers
- [ ] Creating figures
- [ ] Presenting work
- [ ] Open-source projects

**Areas <3: Review relevant chapters!**

---

## 🎯 Transition to Part 3: For Experts

**Part 3 covers:**
- Advanced control theory
- Optimization methods
- State estimation
- Hardware design
- Production deployment
- Cutting-edge research

**Prerequisites:**
- Solid understanding of Part 2
- Linear algebra proficiency
- Calculus and optimization
- Programming fluency

---

## 💡 Tips for Success

### For Research
1. **Start small** - Don't tackle everything at once
2. **Read papers** - 5-10 papers per week in your area
3. **Implement baselines** - Understand state-of-the-art
4. **Keep notes** - Document everything
5. **Collaborate** - Join research labs or teams

### For Development
1. **Version control** - Use git religiously
2. **Write tests** - Test-driven development
3. **Document code** - Your future self will thank you
4. **Modular design** - Keep components independent
5. **Continuous integration** - Automate testing

### For Learning
1. **Build projects** - Theory + practice
2. **Teach others** - Best way to learn
3. **Join communities** - ROS Discourse, Reddit r/robotics
4. **Attend conferences** - Network and learn
5. **Stay curious** - Technology evolves rapidly

---

## 📖 Recommended Reading

**Books:**
- "Probabilistic Robotics" by Thrun et al.
- "Modern Robotics" by Lynch & Park
- "Reinforcement Learning" by Sutton & Barto
- "Deep Learning" by Goodfellow et al.

**Papers:**
- Read recent ICRA/IROS proceedings
- Follow arXiv.org/list/cs.RO daily
- Track citations of key papers in your area

**Online:**
- ROS2 Documentation
- PyTorch Tutorials
- OpenCV Docs
- Isaac Sim Manual

---

## 🏁 Ready for Part 3?

**You're ready if you can:**
- ✅ Build complete robot systems in simulation
- ✅ Implement SLAM and navigation from scratch
- ✅ Train RL agents for robot tasks
- ✅ Design and conduct experiments
- ✅ Write technical documentation
- ✅ Debug complex multi-component systems

**If yes:** Proceed to Chapter 23!

**If not:** Spend more time on projects, review difficult chapters, and practice!

---

## 🎉 Congratulations!

You've completed the **Researcher** level!

You now have the skills to:
- Contribute to robotics research
- Publish papers at top conferences
- Build advanced robot systems
- Lead robotics projects
- Mentor others in robotics

**Next:** Part 3 takes you to the cutting edge of robotics research and industry!

---

**See you in Chapter 23: Advanced Control Theory!** 🚀
`
  },

  // ==========================================
  // PART 3: FOR EXPERTS (15 Chapters, Pages 63-112)
  // ==========================================

  {
    id: 23,
    title: "Advanced Control Theory",
    part: 3,
    partName: "For Experts",
    startPage: 63,
    endPage: 66,
    difficulty: "Hard",
    estimatedTime: "60 minutes",
    objectives: [
      "Master modern control techniques",
      "Implement model predictive control",
      "Design robust controllers"
    ],
    keywords: ["Control Theory", "MPC", "LQR", "Robust Control", "Nonlinear Control"],
    content: `
# Chapter 23: Advanced Control Theory

## 🎛️ Mastering Robot Control

Advanced control theory enables precise, robust robot behavior in complex scenarios.

**Key Topics:**
- Linear Quadratic Regulator (LQR)
- Model Predictive Control (MPC)
- Sliding Mode Control
- Adaptive Control

**In this chapter, you'll learn how to design optimal controllers that handle constraints, uncertainties, and nonlinear dynamics.**

---

## 📐 Linear Quadratic Regulator (LQR)

LQR is an optimal controller for linear systems that minimizes a quadratic cost function.

**Python Implementation:**

\`\`\`python
import numpy as np
from scipy.linalg import solve_continuous_are

class LQRController:
    def __init__(self, A, B, Q, R):
        # Solve Algebraic Riccati Equation
        P = solve_continuous_are(A, B, Q, R)
        # Compute optimal gain
        self.K = np.linalg.inv(R) @ B.T @ P
    
    def control(self, x, x_desired):
        error = x - x_desired
        u = -self.K @ error
        return u
\`\`\`

---

## 🎯 Model Predictive Control (MPC)

MPC solves an optimization problem at each time step to find the best control sequence.

**Benefits:**
- Handles constraints naturally
- Predicts future behavior
- Optimal over finite horizon

---

## 📚 Summary

✅ LQR for optimal linear control
✅ MPC handles constraints and predictions
✅ Advanced methods for complex systems

**Next:** Chapter 24 - Trajectory Optimization
`
  },

  {
    id: 24,
    title: "Trajectory Optimization",
    part: 3,
    partName: "For Experts",
    startPage: 67,
    endPage: 70,
    difficulty: "Hard",
    estimatedTime: "55 minutes",
    objectives: [
      "Formulate optimal control problems",
      "Implement trajectory optimization",
      "Use optimization libraries"
    ],
    keywords: ["Trajectory Optimization", "Direct Collocation", "IPOPT", "Optimal Control"],
    content: `
# Chapter 24: Trajectory Optimization

## 🎢 Finding Optimal Paths

Trajectory optimization computes the best motion to achieve a goal while minimizing cost and satisfying constraints.

**Methods:**
- Direct Collocation
- Shooting Methods
- Differential Dynamic Programming

---

## 🎯 Direct Collocation

Direct collocation discretizes the trajectory and optimizes over all states and controls simultaneously.

**Advantages:**
- Handles constraints well
- Good convergence properties
- Works for complex systems

**Python Example with CasADi:**

\`\`\`python
import casadi as ca

# Create optimization problem
opti = ca.Opti()

# Decision variables
N = 50  # number of time steps
X = opti.variable(4, N+1)  # states
U = opti.variable(2, N)     # controls

# Minimize control effort
cost = ca.sumsqr(U)
opti.minimize(cost)

# Dynamics constraints
for k in range(N):
    x_next = X[:, k] + dt * dynamics(X[:, k], U[:, k])
    opti.subject_to(X[:, k+1] == x_next)

# Solve
opti.solver('ipopt')
sol = opti.solve()
\`\`\`

---

## 📚 Summary

✅ Direct collocation for constrained problems
✅ Optimize over entire trajectory
✅ Use CasADi or Drake for implementation

**Next:** Chapter 25 - State Estimation
`
  },

  {
    id: 25,
    title: "State Estimation and Filtering",
    part: 3,
    partName: "For Experts",
    startPage: 71,
    endPage: 74,
    difficulty: "Hard",
    estimatedTime: "50 minutes",
    objectives: [
      "Implement Kalman filters",
      "Use particle filters",
      "Fuse multiple sensors"
    ],
    keywords: ["Kalman Filter", "EKF", "UKF", "Particle Filter", "Sensor Fusion"],
    content: `
# Chapter 25: State Estimation and Kalman Filtering

## 🔮 Estimating Hidden States

State estimation combines noisy measurements with system models to estimate the true state.

**Filter Types:**
- Kalman Filter (linear systems)
- Extended Kalman Filter (nonlinear)
- Unscented Kalman Filter (highly nonlinear)
- Particle Filter (non-Gaussian)

---

## 📊 Kalman Filter

The Kalman filter is optimal for linear Gaussian systems.

**Python Implementation:**

\`\`\`python
import numpy as np

class KalmanFilter:
    def __init__(self, A, B, H, Q, R, P0):
        self.A = A  # State transition
        self.B = B  # Control input
        self.H = H  # Measurement
        self.Q = Q  # Process noise
        self.R = R  # Measurement noise
        self.P = P0 # Covariance
        self.x = np.zeros(A.shape[0])
    
    def predict(self, u):
        self.x = self.A @ self.x + self.B @ u
        self.P = self.A @ self.P @ self.A.T + self.Q
    
    def update(self, z):
        y = z - self.H @ self.x  # Innovation
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ self.H) @ self.P
\`\`\`

---

## 🎯 Extended Kalman Filter (EKF)

EKF linearizes nonlinear systems for each update.

**Use Cases:**
- Robot localization
- GPS/IMU fusion
- SLAM

---

## 📚 Summary

✅ Kalman filter for linear systems
✅ EKF for mildly nonlinear systems
✅ UKF for highly nonlinear systems
✅ Particle filters for non-Gaussian distributions

**Next:** Chapter 26 - Hardware Integration
`
  },

  {
    id: 26,
    title: "Hardware Integration",
    part: 3,
    partName: "For Experts",
    startPage: 75,
    endPage: 78,
    difficulty: "Hard",
    estimatedTime: "50 minutes",
    objectives: [
      "Design electronics systems",
      "Interface motors and sensors",
      "Implement power management"
    ],
    keywords: ["Electronics", "Motor Drivers", "Power Systems", "CAN Bus", "Hardware"],
    content: `
# Chapter 26: Hardware Integration and Electronics

## ⚡ Building Real Robots

Hardware integration is where theory meets reality!

**Topics:**
- Motor control (BLDC, servo)
- Power management
- Communication protocols (CAN, UART, I2C)
- PCB design basics

---

## 🔌 Motor Control

**Brushless DC (BLDC) Motor Control:**

Key concepts:
- Field-Oriented Control (FOC)
- Electronic Speed Controllers (ESC)
- Current sensing
- Position feedback (encoders)

**C++ Example (STM32):**

\`\`\`cpp
class BLDCController {
public:
    void controlLoop() {
        // Read encoder
        float angle = readEncoder();
        float velocity = calculateVelocity(angle);
        
        // PI controller
        float error = target_velocity - velocity;
        float torque_cmd = kp * error + ki * integral;
        
        // Apply FOC
        applyFOC(angle, torque_cmd);
    }
};
\`\`\`

---

## 🔋 Power Management

**Battery Management:**
- Voltage monitoring
- Current limiting
- Temperature protection
- State of Charge (SOC) estimation

**Safety Features:**
- Emergency stop circuit
- Overcurrent protection
- Thermal shutdown

---

## 📡 Communication Protocols

**CAN Bus:**
- Real-time communication
- Multi-device network
- Error detection
- Priority-based arbitration

**I2C:**
- Sensor communication
- Simple 2-wire interface
- Multiple devices on one bus

---

## 📚 Summary

✅ FOC for smooth motor control
✅ Battery management for safety
✅ CAN bus for real-time communication
✅ Proper PCB design prevents issues

**Next:** Chapter 27 - Production Deployment
`
  },

  {
    id: 27,
    title: "Production Deployment",
    part: 3,
    partName: "For Experts",
    startPage: 79,
    endPage: 82,
    difficulty: "Hard",
    estimatedTime: "45 minutes",
    objectives: [
      "Deploy robots in production",
      "Implement monitoring and logging",
      "Handle failures gracefully"
    ],
    keywords: ["Deployment", "Production", "Monitoring", "DevOps", "Reliability"],
    content: `
# Chapter 27: Production Deployment

## 🚀 From Lab to Real World

Deploying robots in production requires robustness, monitoring, and failure handling.

**Topics:**
- Containerization (Docker)
- Monitoring and telemetry
- Logging strategies
- Remote management
- Security

---

## 📦 Containerization

**Docker for Robotics:**

\`\`\`dockerfile
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    ros-humble-navigation2

# Copy code
WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/

# Build
RUN . /opt/ros/humble/setup.sh && \\
    colcon build

CMD ["ros2", "launch", "my_robot", "robot.launch.py"]
\`\`\`

**Docker Compose for Multi-Container:**

\`\`\`yaml
version: '3.8'
services:
  navigation:
    build: ./navigation
    network_mode: host
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    restart: unless-stopped
  
  perception:
    build: ./perception
    runtime: nvidia
    restart: unless-stopped
\`\`\`

---

## 📊 Monitoring

**Prometheus Metrics:**

\`\`\`python
from prometheus_client import Counter, Gauge, start_http_server

msg_count = Counter('ros_messages_total', 'Messages processed')
queue_size = Gauge('ros_queue_size', 'Current queue size')

# Start metrics server
start_http_server(8000)
\`\`\`

**System Health Dashboard:**
- CPU/Memory usage
- Message rates
- Error counts
- Robot state

---

## 🔄 Automatic Recovery

**Watchdog System:**

\`\`\`python
class WatchdogNode(Node):
    def __init__(self):
        super().__init__('watchdog')
        self.last_heartbeat = {}
        self.timeout = 5.0
        
    def check_nodes(self):
        current_time = self.get_clock().now()
        for node_name, last_time in self.last_heartbeat.items():
            if (current_time - last_time).nanoseconds / 1e9 > self.timeout:
                self.restart_node(node_name)
\`\`\`

---

## 📚 Summary

✅ Docker for reproducible deployments
✅ Prometheus for monitoring
✅ Automatic recovery with watchdogs
✅ Structured logging for debugging

**Next:** Chapter 28 - Performance Optimization
`
  }
,

  {
    id: 28,
    title: "Performance Optimization",
    part: 3,
    partName: "For Experts",
    startPage: 83,
    endPage: 86,
    difficulty: "Hard",
    estimatedTime: "40 minutes",
    objectives: [
      "Profile and optimize code",
      "Reduce latency",
      "Improve throughput"
    ],
    keywords: ["Optimization", "Profiling", "Performance", "Latency", "Efficiency"],
    content: `
# Chapter 28: Performance Optimization

## ⚡ Making Robots Faster

Performance optimization is critical for real-time robotics.

**Techniques:**
- Profiling and benchmarking
- Vectorization
- Multi-threading
- GPU acceleration
- Memory optimization

---

## 📊 Profiling

**Python Profiling:**

\`\`\`python
import cProfile
import pstats

profiler = cProfile.Profile()
profiler.enable()

# Run code
rclpy.spin(node)

profiler.disable()
stats = pstats.Stats(profiler)
stats.sort_stats('cumulative')
stats.print_stats(20)
\`\`\`

---

## 🚀 Optimization Techniques

**1. Vectorization with NumPy:**

\`\`\`python
# Slow (loop)
result = [x**2 + 2*x + 1 for x in data]

# Fast (vectorized)
result = data**2 + 2*data + 1  # 100x faster!
\`\`\`

**2. Multi-threading:**

\`\`\`python
from concurrent.futures import ThreadPoolExecutor

executor = ThreadPoolExecutor(max_workers=4)
futures = [executor.submit(process_image, img) for img in images]
results = [f.result() for f in futures]
\`\`\`

**3. GPU Acceleration:**

\`\`\`python
import cupy as cp

# Move to GPU
data_gpu = cp.asarray(data)
result_gpu = cp.fft.fft2(data_gpu)
result = cp.asnumpy(result_gpu)
\`\`\`

---

## 📚 Summary

✅ Profile before optimizing
✅ Vectorize with NumPy/CuPy
✅ Use multi-threading for I/O
✅ GPU for heavy computation

**Next:** Chapter 29 - Cutting-Edge Research
`
  },

  {
    id: 29,
    title: "Cutting-Edge Research Topics",
    part: 3,
    partName: "For Experts",
    startPage: 87,
    endPage: 90,
    difficulty: "Hard",
    estimatedTime: "55 minutes",
    objectives: [
      "Explore frontier research",
      "Understand emerging technologies",
      "Identify future directions"
    ],
    keywords: ["Research", "Foundation Models", "Sim-to-Real", "Embodied AI", "Future"],
    content: `
# Chapter 29: Cutting-Edge Research Topics

## 🔬 The Future of Physical AI

Explore the bleeding edge of robotics research!

**Hot Topics:**
- Foundation models for robotics
- Sim-to-real transfer
- Imitation learning from videos
- Neural architecture search
- Diffusion models for planning

---

## 🤖 Foundation Models for Robotics

**Vision-Language-Action (VLA) Models:**

Unified models that understand vision, language, and generate actions.

**Examples:**
- RT-2 (Robotics Transformer 2) by Google
- PaLM-E by Google
- Gato by DeepMind

**Key Idea:** One model for all tasks!

---

## 🌍 Sim-to-Real Transfer

**Domain Randomization 2.0:**

Automatically expand randomization ranges based on policy performance.

**Techniques:**
- Automatic Domain Randomization (ADR)
- Adversarial randomization
- System identification
- Progressive training

---

## 🧠 Learning from Videos

**Learning manipulation from YouTube:**

Train robots by watching human demonstrations online.

**Challenges:**
- Viewpoint differences
- Hand-to-robot mapping
- Understanding intent

---

## 🎨 Diffusion Models

**Diffusion Policy:**

Generate smooth, multi-modal action distributions.

**Advantages:**
- Handles multi-modal data
- Generates smooth trajectories
- State-of-the-art results

---

## 📚 Summary

✅ Foundation models enable generalist robots
✅ Sim-to-real gap is closing
✅ Learning from videos scales data
✅ Diffusion models generate smooth motions

**Next:** Chapter 30 - Building Your Career
`
  },

  {
    id: 30,
    title: "Building Your Robotics Career",
    part: 3,
    partName: "For Experts",
    startPage: 91,
    endPage: 93,
    difficulty: "Hard",
    estimatedTime: "30 minutes",
    objectives: [
      "Navigate career paths",
      "Build strong portfolio",
      "Network effectively"
    ],
    keywords: ["Career", "Industry", "Academia", "Portfolio", "Networking"],
    content: `
# Chapter 30: Building Your Robotics Career

## 🚀 Your Path in Robotics

From student to expert - build a successful career!

**Career Paths:**
1. Industry Research Scientist ($150k-$400k+)
2. Robotics Engineer ($100k-$200k)
3. ML Engineer - Robotics ($120k-$250k)
4. Academic Professor ($80k-$180k)
5. Startup Founder ($$$$)

---

## 📁 Building Your Portfolio

**GitHub Projects:**
- 3-5 quality projects
- Well-documented
- Working demos (videos!)
- Clean code

**Example Project Structure:**

\`\`\`
robot-navigation/
├── README.md       # Clear explanation
├── demos/          # GIFs/videos
├── src/            # Clean code
├── tests/          # Unit tests
└── requirements.txt
\`\`\`

---

## 🤝 Networking

**Top Conferences:**
- ICRA (May)
- IROS (September/October)
- RSS (July)
- CoRL (November)

**Online Communities:**
- ROS Discourse
- r/robotics
- Twitter robotics community
- Discord servers

---

## 📝 Resume Tips

**Format:**
- Quantify impact: "Improved speed by 40%"
- Highlight tech: "Implemented SLAM using ROS2"
- Show results: "Deployed to 50+ robots"

---

## 🎤 Interview Prep

**Technical Topics:**
- Kinematics (FK/IK)
- Control theory (PID, MPC)
- Path planning (A*, RRT)
- SLAM algorithms
- Machine learning basics

**Common Questions:**
- "Explain how an EKF works"
- "How would you implement obstacle avoidance?"
- "Debug this ROS2 launch file"

---

## 📚 Summary

✅ Multiple career paths available
✅ Portfolio is crucial
✅ Networking accelerates opportunities
✅ Continuous learning essential

**Next:** Chapter 31 - Advanced Manipulation
`
  },

  {
    id: 31,
    title: "Advanced Manipulation Techniques",
    part: 3,
    partName: "For Experts",
    startPage: 94,
    endPage: 97,
    difficulty: "Hard",
    estimatedTime: "45 minutes",
    objectives: [
      "Master dexterous manipulation",
      "Handle contact-rich tasks",
      "Work with deformable objects"
    ],
    keywords: ["Dexterous Manipulation", "Contact Dynamics", "Grasping", "Compliance"],
    content: `
# Chapter 31: Advanced Manipulation Techniques

## 🦾 Mastering Complex Manipulation

Beyond simple pick-and-place!

**Topics:**
- Dexterous hand control
- Compliance control
- Contact-rich manipulation
- Deformable objects
- In-hand manipulation

---

## 🖐️ Dexterous Grasp Planning

**Multi-Fingered Grasping:**

Find optimal contact points for stable grasp.

**Quality Metrics:**
- Force closure
- Ferrari-Canny metric
- Minimum singular value

**Python Implementation:**

\`\`\`python
class GraspPlanner:
    def evaluate_grasp_quality(self, contacts):
        G = self.build_grasp_matrix(contacts)
        U, s, Vt = np.linalg.svd(G)
        quality = np.min(s)  # Ferrari-Canny metric
        return quality
\`\`\`

---

## 🔧 Impedance Control

**Compliant Manipulation:**

Control the relationship between force and position.

\`\`\`python
class ImpedanceController:
    def compute_control(self, x, x_d, F_ext):
        # Desired impedance: M*ddx + B*dx + K*x = F_ext
        ddx_desired = (F_ext - self.B @ (dx - dx_d) - self.K @ (x - x_d)) / self.M
        return ddx_desired
\`\`\`

**Applications:**
- Peg-in-hole insertion
- Polishing/grinding
- Human collaboration

---

## 📦 Contact-Rich Manipulation

**Handling Multiple Contacts:**

Estimate and control contact forces at multiple points.

**Key Concepts:**
- Contact mode estimation
- Hybrid force-position control
- Friction cone constraints

---

## 🧵 Deformable Objects

**Cable/Rope Manipulation:**

\`\`\`python
class CableManipulator:
    def plan_cable_routing(self, start, end, obstacles):
        # Use RRT for geometric path
        path = self.rrt_path_planning(start, end, obstacles)
        
        # Smooth considering cable dynamics
        smoothed = self.smooth_cable_path(path)
        return smoothed
\`\`\`

---

## 📚 Summary

✅ Dexterous grasping with quality metrics
✅ Impedance control for compliance
✅ Contact-rich task handling
✅ Deformable object manipulation

**Next:** Chapter 32 - Humanoid Locomotion
`
  },

  {
    id: 32,
    title: "Humanoid Locomotion",
    part: 3,
    partName: "For Experts",
    startPage: 98,
    endPage: 101,
    difficulty: "Hard",
    estimatedTime: "50 minutes",
    objectives: [
      "Implement bipedal walking",
      "Handle dynamic balance",
      "Navigate complex terrain"
    ],
    keywords: ["Bipedal", "Locomotion", "ZMP", "Whole-Body Control", "Walking"],
    content: `
# Chapter 32: Humanoid Locomotion

## 🚶 Walking Like a Human

Bipedal locomotion is one of the hardest problems in robotics!

**Key Concepts:**
- Zero Moment Point (ZMP)
- Whole-body control
- Dynamic walking
- Terrain adaptation
- Fall recovery

---

## ⚖️ Zero Moment Point (ZMP)

**ZMP-based Walking:**

ZMP is the point where net moment is zero.

\`\`\`python
class ZMPController:
    def compute_zmp(self, com_pos, com_acc):
        x_zmp = com_pos[0] - (com_pos[2] / g) * com_acc[0]
        y_zmp = com_pos[1] - (com_pos[2] / g) * com_acc[1]
        return np.array([x_zmp, y_zmp])
    
    def is_stable(self, zmp, support_polygon):
        # Check if ZMP inside support polygon
        from shapely.geometry import Point, Polygon
        return Polygon(support_polygon).contains(Point(zmp))
\`\`\`

---

## 🦿 Whole-Body Control

**Quadratic Program for Locomotion:**

Solve for joint accelerations and contact forces.

\`\`\`python
import cvxpy as cp

# Decision variables
ddq = cp.Variable(num_joints)  # Joint accelerations
F = cp.Variable(num_contacts * 3)  # Contact forces

# Minimize: ||ddq - ddq_desired||²
# Subject to: M*ddq + h = J^T*F (dynamics)
#             F in friction cone
\`\`\`

---

## 🏃 Dynamic Walking with MPC

**Model Predictive Control for Gait:**

\`\`\`python
class MPCLocomotion:
    def plan_gait(self, current_state, target_velocity):
        # Optimize over horizon
        # Decision vars: CoM trajectory, footsteps
        # Cost: track velocity, minimize effort
        # Constraints: dynamics, reachability
        return footsteps, com_trajectory
\`\`\`

---

## 🌄 Terrain Adaptation

**Adaptive Gait Parameters:**

Adjust step height, length, and frequency based on terrain type.

\`\`\`python
def adapt_gait(terrain_type):
    params = {
        'flat': {'step_height': 0.05, 'step_length': 0.3},
        'stairs': {'step_height': 0.2, 'step_length': 0.25},
        'uneven': {'step_height': 0.15, 'step_length': 0.2}
    }
    return params[terrain_type]
\`\`\`

---

## 📚 Summary

✅ ZMP for stable walking
✅ Whole-body QP control
✅ MPC for dynamic gaits
✅ Terrain adaptation
✅ Fall recovery

**Next:** Chapter 33 - Sim-to-Real Transfer
`
  }
,

  {
    id: 33,
    title: "Sim-to-Real Transfer Mastery",
    part: 3,
    partName: "For Experts",
    startPage: 102,
    endPage: 105,
    difficulty: "Hard",
    estimatedTime: "45 minutes",
    objectives: [
      "Bridge simulation-reality gap",
      "Implement domain randomization",
      "Measure transfer success"
    ],
    keywords: ["Sim-to-Real", "Domain Randomization", "Reality Gap", "Transfer"],
    content: `
# Chapter 33: Sim-to-Real Transfer Mastery

## 🌉 Bridging the Reality Gap

Training in simulation, deploying to reality!

**Common Failures:**
- Physics mismatch
- Sensor noise differences
- Actuator dynamics
- Environment variations

**Solution: Make simulation MORE diverse than reality!**

---

## 🎲 System Identification

**Measuring Real Robot Parameters:**

\`\`\`python
class SystemID:
    def identify_friction(self, joint):
        # Gradually increase torque until motion
        torque = 0
        while not self.detect_motion(joint):
            torque += 0.1
            joint.apply_torque(torque)
        return torque  # Static friction
\`\`\`

---

## 🔧 Advanced Domain Randomization

**Curriculum-Based Randomization:**

\`\`\`python
class CurriculumDR:
    def step(self, policy_performance):
        recent_avg = np.mean(self.performance_history[-100:])
        
        if recent_avg > 0.95:
            self.expand_ranges(factor=1.1)
        elif recent_avg < 0.7:
            self.contract_ranges(factor=0.95)
\`\`\`

---

## 📊 Measuring Transfer Success

**Transfer Metrics:**

\`\`\`python
class TransferEvaluator:
    def evaluate_transfer(self, policy, sim_env, real_robot):
        sim_success = self.evaluate(policy, sim_env, n=100)
        real_success = self.evaluate(policy, real_robot, n=20)
        
        transfer_gap = sim_success - real_success
        transfer_ratio = real_success / sim_success
        
        return {'gap': transfer_gap, 'ratio': transfer_ratio}
\`\`\`

---

## 📚 Summary

✅ System ID measures real parameters
✅ Curriculum DR expands gradually
✅ Transfer metrics quantify success
✅ Safe fine-tuning improves performance

**Next:** Chapter 34 - Ethical AI
`
  },

  {
    id: 34,
    title: "Ethical AI and Responsible Robotics",
    part: 3,
    partName: "For Experts",
    startPage: 106,
    endPage: 108,
    difficulty: "Hard",
    estimatedTime: "35 minutes",
    objectives: [
      "Design ethically-aware systems",
      "Implement fairness",
      "Address societal impacts"
    ],
    keywords: ["Ethics", "Fairness", "Transparency", "Accountability", "Bias"],
    content: `
# Chapter 34: Ethical AI and Responsible Robotics

## 🤔 Building Robots We Can Trust

Ethics must be engineered from day one!

**Key Principles:**
1. Safety First
2. Transparency
3. Fairness
4. Privacy
5. Accountability

---

## ⚖️ Fairness in Robot Behavior

**Checking for Bias:**

\`\`\`python
class FairnessChecker:
    def check_fairness(self, policy, test_scenarios):
        results_by_group = {}
        
        for scenario in test_scenarios:
            group = scenario['demographic_group']
            success = policy.execute(scenario)
            results_by_group[group].append(success)
        
        # Check demographic parity
        success_rates = {g: np.mean(r) for g, r in results_by_group.items()}
        max_disparity = max(success_rates.values()) - min(success_rates.values())
        
        return max_disparity < 0.1  # 10% threshold
\`\`\`

---

## 🔍 Explainable Decisions

**Explain Robot Actions:**

\`\`\`python
def explain_action(observation, action):
    # Use SHAP for feature importance
    importances = compute_feature_importance(observation)
    
    explanation = "I chose this action because:\\n"
    for feature, importance in top_features:
        explanation += f"- {feature}: {importance:.1%} influence\\n"
    
    return explanation
\`\`\`

---

## 📝 Accountability

**Decision Logging:**

\`\`\`python
class AccountabilitySystem:
    def log_decision(self, decision_data):
        log_entry = {
            'timestamp': time.time(),
            'observation': decision_data['observation'],
            'action': decision_data['action'],
            'explanation': decision_data['explanation']
        }
        self.decision_log.append(log_entry)
        self.save_to_database(log_entry)
\`\`\`

---

## 📚 Summary

✅ Engineer fairness into systems
✅ Make decisions explainable
✅ Maintain accountability through logging
✅ Assess societal impacts

**Next:** Chapter 35 - The Future
`
  },

  {
    id: 35,
    title: "The Future of Physical AI",
    part: 3,
    partName: "For Experts",
    startPage: 109,
    endPage: 111,
    difficulty: "Hard",
    estimatedTime: "30 minutes",
    objectives: [
      "Explore emerging trends",
      "Predict future developments",
      "Prepare for what's next"
    ],
    keywords: ["Future", "Trends", "Predictions", "Innovation", "Breakthroughs"],
    content: `
# Chapter 35: The Future of Physical AI

## 🔮 What's Coming Next?

The future is being written NOW!

---

## 🚀 Near-Term (1-3 years)

**1. Foundation Models Go Multimodal**

One model handles vision, language, and control!

**2. Sim-to-Real Becomes Trivial**

Train once, deploy everywhere.

**3. Humanoid Robots in Homes**

<$20k consumer robots by 2025-2026.

**Companies:**
- Figure AI (Figure 02)
- 1X Technologies (NEO)
- Tesla (Optimus)
- Sanctuary AI (Phoenix)

---

## 🌟 Medium-Term (3-7 years)

**1. Dexterous Manipulation Solved**

Human-level manipulation capability.

**2. Collaborative Robots Everywhere**

From factories to homes.

**3. Edge AI for Robotics**

Fully onboard intelligence.

---

## 🚁 Long-Term (7-15 years)

**1. General-Purpose Humanoids**

Learn any physical task from video.

**2. Swarm Robotics at Scale**

Thousands of robots coordinating.

**3. Brain-Computer Interfaces**

Direct neural control.

---

## 💡 Wild Predictions

**Experts Say:**
- "Robotics 1000x harder BUT 1000x more impactful"
- "In 10 years, manipulation = human level"
- "By 2030, personal robots = common as dishwashers"

---

## 🔬 Open Research Problems

1. Long-horizon planning
2. Continual learning
3. Common sense reasoning
4. Energy efficiency
5. Self-repair

---

## 📚 Summary

✅ Humanoids coming to market soon
✅ Foundation models revolutionize robotics
✅ Major unsolved problems remain
✅ Enormous career opportunities

**Next:** Chapter 36 - Final Projects
`
  },

  {
    id: 36,
    title: "Capstone Project Ideas",
    part: 3,
    partName: "For Experts",
    startPage: 112,
    endPage: 112,
    difficulty: "Hard",
    estimatedTime: "20 minutes",
    objectives: [
      "Design ambitious projects",
      "Integrate all skills",
      "Create portfolio work"
    ],
    keywords: ["Projects", "Capstone", "Portfolio", "Integration", "Innovation"],
    content: `
# Chapter 36: Capstone Project Ideas

## 🎯 Your Masterpiece

Build something extraordinary!

---

## 🏆 Expert-Level Projects

### Project 1: Full-Stack Humanoid Robot

**Goal:** Build complete humanoid from scratch

**Components:**
- Custom mechanical design
- Electronics and motor drivers
- Low-level control (real-time)
- High-level planning (ROS2)
- Vision and SLAM
- Learning from demonstration

**Timeline:** 6-12 months
**Difficulty:** ⭐⭐⭐⭐⭐

---

### Project 2: Foundation Model for Robotics

**Goal:** Train generalist robot policy

**Approach:**
- Collect diverse dataset
- Train vision-language-action model
- Zero-shot task transfer
- Benchmark on standard tasks

**Timeline:** 6-9 months
**Could be your thesis or startup!**

---

### Project 3: Autonomous Restaurant Robot

**Goal:** Full-service restaurant robot

**Capabilities:**
- Take orders (speech)
- Navigate busy restaurant
- Deliver food
- Handle edge cases

**Partner with local restaurant!**

---

### Project 4: Open-Source Robot Platform

**Goal:** The "Raspberry Pi of Robotics"

**Requirements:**
- Low-cost (<$500)
- Easy to build
- Well-documented
- Extensible

**Impact: Democratize robotics!**

---

## 📝 Project Planning

### Phase 1: Design (2 weeks)
- Define requirements
- System architecture
- Component selection

### Phase 2: Prototyping (4 weeks)
- Build MVP
- Test core functionality

### Phase 3: Integration (4 weeks)
- Combine subsystems
- End-to-end testing

### Phase 4: Refinement (3 weeks)
- Polish UX
- Handle edge cases

### Phase 5: Documentation (1 week)
- Technical report
- Video demo
- Publish code

---

## 🎬 Showcasing Your Work

**Create:**
- 2-minute demo video
- Clean GitHub repo
- Blog post
- Conference submission

---

## 📚 Summary

✅ Expert projects integrate everything
✅ Real-world deployment is the goal
✅ Document and showcase well

**Next:** Chapter 37 - Conclusion
`
  },

  {
    id: 37,
    title: "Conclusion and Resources",
    part: 3,
    partName: "For Experts",
    startPage: 112,
    endPage: 112,
    difficulty: "Hard",
    estimatedTime: "15 minutes",
    objectives: [
      "Review complete journey",
      "Access ongoing resources",
      "Join robotics community"
    ],
    keywords: ["Conclusion", "Resources", "Community", "Next Steps", "Learning"],
    content: `
# Chapter 37: Conclusion and Resources

## 🎓 You Made It!

From beginner to expert - incredible journey!

---

## 📖 What You've Mastered

### Part 1: Students (Chapters 1-10)
✅ ROS2, Linux, URDF
✅ Sensors, kinematics, navigation
✅ First robot project

### Part 2: Researchers (Chapters 11-22)
✅ Advanced simulation
✅ Computer vision, SLAM
✅ Motion planning, RL
✅ Multi-robot systems
✅ HRI, safety, publishing

### Part 3: Experts (Chapters 23-37)
✅ Advanced control, optimization
✅ State estimation, hardware
✅ Production deployment
✅ Cutting-edge research
✅ Career development
✅ Advanced manipulation
✅ Humanoid locomotion

---

## 📚 Essential Resources

**Books:**
- "Modern Robotics" - Lynch & Park
- "Probabilistic Robotics" - Thrun et al.
- "Planning Algorithms" - LaValle

**Online Courses:**
- MIT OpenCourseWare
- Coursera - Robotics Specialization

**Documentation:**
- ROS2: docs.ros.org
- MoveIt2: moveit.ros.org
- PyTorch: pytorch.org/tutorials

**Communities:**
- ROS Discourse
- r/robotics
- Twitter #robotics

**Conferences:**
- ICRA (May)
- IROS (Oct)
- RSS (July)
- CoRL (Nov)

---

## 🎯 Your Action Plan

### Next 30 Days
- [ ] Pick capstone project
- [ ] Set up environment
- [ ] Start building

### Next 3 Months
- [ ] Complete project
- [ ] Document on GitHub
- [ ] Create demo video

### Next Year
- [ ] Attend conference
- [ ] Publish paper/blog
- [ ] Land dream job
- [ ] Mentor others

---

## 💌 Final Message

**Congratulations!** You've completed a comprehensive journey through Physical AI and Humanoid Robotics.

But this isn't the end - it's just the beginning!

**Remember:**
- Robotics is hard - embrace challenges
- Failure is learning
- Community helps
- Build things
- Share your work
- Stay curious
- Have fun!

---

## 🚀 Where Will You Go?

Will you:
- Join a top robotics lab?
- Start a robotics company?
- Pursue a PhD?
- Build open-source tools?
- Teach others?

**Whatever path you choose, the world needs your skills!**

---

## 🙏 Thank You

Thank you for investing your time in learning robotics.

**Your journey doesn't end here - it evolves!**

> "The best way to predict the future is to invent it." - Alan Kay

**You now have the tools to invent the future of robotics.**

---

**Go forth and build amazing robots!** 🚀🤖✨

---

**THE END**

*Physical AI and Humanoid Robotics*
*by Azmat Ali*

---

**Now go build the future!** 🎉
`
  }
];

// Utility function to get chapter by ID
export const getChapterById = (id: number): Chapter | undefined => {
  return bookData.find(chapter => chapter.id === id);
};

// Utility function to get chapters by part
export const getChaptersByPart = (part: 1 | 2 | 3): Chapter[] => {
  return bookData.filter(chapter => chapter.part === part);
};

// Utility function to get all keywords for search
export const getAllKeywords = (): string[] => {
  const keywords = new Set<string>();
  bookData.forEach(chapter => {
    chapter.keywords.forEach(keyword => keywords.add(keyword));
  });
  return Array.from(keywords);
};

export const bookMetadata = {
  title: "Physical AI and Humanoid Robotics",
  author: "Azmat Ali",
  totalChapters: 37,
  totalPages: 112,
  parts: [
    { number: 1, name: "For Students", chapters: 10, pages: 27, difficulty: "Easy" },
    { number: 2, name: "For Researchers", chapters: 12, pages: 35, difficulty: "Medium" },
    { number: 3, name: "For Experts", chapters: 15, pages: 50, difficulty: "Hard" }
  ]
};