# 🚀 AUCTUS: Smart Delivering Robot - Graduation Project 🤖📦

<div align="center">
  <img src="https://img.shields.io/badge/Status-Completed-brightgreen" alt="Status">
  <img src="https://img.shields.io/badge/Platform-ROS%20Melodic-blue" alt="Platform">
  <img src="https://img.shields.io/badge/Hardware-Arduino%20Mega-orange" alt="Hardware">
  <img src="https://img.shields.io/badge/License-MIT-yellow" alt="License">
</div>

---

## 📌 Overview  
**AUCTUS** is an **autonomous delivering robot** designed to address traditional logistics inefficiencies by providing **accurate navigation, eco-friendly delivery, and intelligent obstacle avoidance**. The robot operates via a **user-friendly mobile and web interface**, making it ideal for outdoor and indoor delivery applications. This project was developed as part of our graduation project and aims to contribute to the future of autonomous delivery.

---

## 🎯 Objectives  
- **Eco-Friendly Design:** Zero carbon emissions using battery-powered operation.  
- **Real-Time Tracking and Monitoring:** Continuous tracking via a web interface for location and delivery status.  
- **Energy Efficiency:** Optimized battery system with long-duration performance and quick recharging.  
- **User Authentication and Security:** QR-based authentication for secure access to deliveries.  
- **Autonomous Navigation and Obstacle Avoidance:** Intelligent obstacle avoidance using LiDAR and SLAM algorithms.  
- **Interactive User Interface:** Easy setup for deliveries, monitoring, and notifications.

---

## 🚀 Features  
✅ **Autonomous navigation** using path planning and SLAM algorithms.  
✅ **Real-time location monitoring** through a web interface.  
✅ **Obstacle detection and avoidance** for safe deliveries.  
✅ **Secure delivery authentication** via QR codes.  
✅ **Battery efficiency and power management** for extended operation.  
✅ **Interactive user interface** for monitoring and control.

---

## 🛠️ Technologies & Components  

| **Category**          | **Components/Technologies**               |
|----------------------|-------------------------------------------|
| **Microcontroller**   | Arduino Mega, LattePanda Delta           |
| **Sensors**           | LiDAR (A1M8 Slamtec), IMU (9-DOF Adafruit)|
| **Actuators**         | DC Motors with Encoders, Servo Motors    |
| **Power Management**  | 12V Ultracell Batteries, DC-DC Converter |
| **Operating System**  | ROS Melodic                              |
| **Software Algorithms**| SLAM, EKF (Extended Kalman Filter), Obstacle Avoidance |
| **Communication**     | Serial Communication (UART)             |
| **Web Interface**     | http://www.cai.aast.edu/auctus           |

---

## 🚀 Installation & Setup

### 1️⃣ Prerequisites
```bash
Ubuntu 18.04 LTS
ROS Melodic
Arduino IDE 1.8+
Git
```

### 2️⃣ Install Required Libraries and Dependencies
**ROS Melodic:** Install on Ubuntu 18.04 using ROS installation guide:
```bash
sudo apt update && sudo apt install ros-melodic-desktop-full
```

**Arduino Libraries:**
```bash
# Arduino Library Manager → Install:
# - "Adafruit 9-DOF IMU"
# - "Motor Driver"
```

### 3️⃣ Clone and Build
```bash
# Clone the repository
git clone https://github.com/yourusername/auctus-delivery-robot.git
cd auctus-delivery-robot

# Build ROS workspace
cd catkin_ws
catkin_make
source devel/setup.bash
```

### 4️⃣ Upload the Code
Connect the Arduino Mega and upload the `main_controller.ino`.

### 5️⃣ Hardware Setup
| **Component** | **Connection Pins** |
|---------------|-------------------|
| LiDAR A1M8 | UART (TX → RX, RX → TX) |
| Motors with Encoders | Motor driver connected to PWM pins |
| 9-DOF IMU | I2C interface |
| Battery | Connected to main power rail |
| Touch Screen & Lights | Power and data pins |

---

## 📌 How It Works  

### 🔍 Step 1: Initial Setup  
- The system initializes sensors and components, calibrating the **IMU** and **LiDAR** for accurate navigation.  

### 🔍 Step 2: User Request and Authentication  
- The user places an order through the **web interface** and scans a **QR code** to authenticate the package retrieval.  

### 🔍 Step 3: Navigation and Obstacle Avoidance  
- The robot uses **SLAM** to create a map of the environment and **path planning algorithms** to find the optimal route.  
- **Obstacle detection** ensures safe navigation by avoiding obstacles in real time using **LiDAR and sensor fusion**.  

### 🔍 Step 4: Delivery  
- The robot navigates to the specified delivery location, using **global and local path planning**.  
- Upon reaching the target, the robot **notifies the user** and allows access via **QR-based authentication**.  

### 🔍 Step 5: Return to Base  
- After successful delivery, the robot **autonomously returns** to its base station for charging or standby mode.  

---

## 🚀 Usage

### Launch the System
```bash
# Terminal 1: Start ROS Core
roscore

# Terminal 2: Launch robot hardware interface
roslaunch auctus_robot robot.launch

# Terminal 3: Start navigation system
roslaunch auctus_navigation navigation.launch

# Terminal 4: Launch web interface
rosrun auctus_web web_server.py
```

### Access Web Interface
- **Local:** `http://localhost:8080`
- **Demo:** `http://www.cai.aast.edu/auctus`

### Basic API Usage
```python
from auctus_robot import DeliveryRobot

# Initialize robot
robot = DeliveryRobot()

# Set destination
robot.set_destination(x=10.0, y=5.0, theta=0.0)

# Start delivery
robot.start_delivery()

# Check status
status = robot.get_status()
print(f"Position: {status['position']}")
print(f"Battery: {status['battery']}%")
```

---

## 📊 Testing & Results  
The system was tested in various indoor and outdoor settings. The results show:  
✅ **Obstacle avoidance accuracy:** 97%  
✅ **Average delivery time:** 5 minutes over 500m  
✅ **Battery performance:** Continuous operation for 1.5 hours with a 30-minute charging interval  
✅ **Delivery success rate:** 94%  

---

## ⚙️ Challenges  

### 🔹 **Navigation in dynamic environments:**  
- Implemented **SLAM** and real-time **obstacle avoidance** to tackle unpredictable spaces.  

### 🔹 **Power management:**  
- Optimized **battery usage** to support longer operational hours.  

### 🔹 **Cost efficiency:**  
- Minimized **component costs** without compromising performance.  

### 🔹 **System security:**  
- Implemented secure access using **QR-based authentication**.  

---

## 🚧 Troubleshooting

### Common Issues
```bash
# LiDAR connection problems
sudo chmod 666 /dev/ttyUSB0
dmesg | tail

# ROS nodes not communicating
rosnode list
rostopic list
roscore  # Restart if needed

# Navigation issues
rosrun rviz rviz  # Visualize in RViz
rosparam get /move_base/  # Check parameters

# Arduino communication
ls /dev/tty*  # List serial ports
```

---

## 🛠️ Future Improvements  
🔹 **AI-powered Path Planning:** Use **machine learning** to predict and optimize delivery routes.  
🔹 **Solar Charging:** Integrate **solar panels** for improved energy efficiency.  
🔹 **Mobile App Integration:** Provide real-time updates through a **dedicated app**.  
🔹 **Payload Capacity:** Expand the storage to **handle larger deliveries**.  

---

## 📂 Project Structure
```
auctus-delivery-robot/
├── catkin_ws/
│   ├── src/
│   │   ├── auctus_robot/          # Main robot package
│   │   ├── auctus_navigation/     # Navigation algorithms
│   │   ├── auctus_web/           # Web interface
│   │   └── auctus_msgs/          # Custom messages
│   └── build/
├── arduino/
│   ├── main_controller/          # Arduino main code
│   └── libraries/                # Required libraries
├── docs/                         # Documentation
├── tests/                        # Test scripts
├── config/                       # Configuration files
└── README.md
```

---

## 🤝 Contributing
We welcome contributions! Please follow these steps:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request


---

## 📧 Contact  
For any inquiries or support, feel free to reach out:  
- **👤 Team Members:** Ahmad Muhammad El-Sayed, Pavli Bahaa Botrus, Yousef Ali Abdallah, Amr Ashraf Fawzy, Abdelrahman Ahmed Mohamed, Ahmed Saad El-Menawy  
- **👨‍🏫 Advisors:** Dr. Omar Shalash, Eng. Mohamed El-Sayed  
- **🏫 Institution:** Arab Academy for Science, Technology & Maritime Transport  

---

<div align="center">
  <p><strong>⭐ If you found this project helpful, please give it a star! ⭐</strong></p>
  <p>Made with ❤️ by AUCTUS Team</p>
</div>
