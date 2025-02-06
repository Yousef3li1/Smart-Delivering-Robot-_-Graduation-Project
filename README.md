# 🚀 AUCTUS: Smart Delivering Robot - Graduation Project 🤖📦

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

## 2️⃣ Install Required Libraries and Dependencies
ROS Melodic: Install on Ubuntu 18.04 using ROS installation guide:
bash
Copy
Edit
sudo apt update && sudo apt install ros-melodic-desktop-full
Arduino Libraries:
bash
Copy
Edit
Arduino Library Manager → Install "Adafruit 9-DOF IMU"
Arduino Library Manager → Install "Motor Driver"
## 3️⃣ Upload the Code
Connect the Arduino Mega and upload the main_controller.ino.
## 4️⃣ Hardware Setup
Component	Connection Pins
LiDAR A1M8	UART (TX → RX, RX → TX)
Motors with Encoders	Motor driver connected to PWM pins
9-DOF IMU	I2C interface
Battery	Connected to main power rail
Touch Screen & Lights	Power and data pins
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

## 🛠️ Future Improvements  
🔹 **AI-powered Path Planning:** Use **machine learning** to predict and optimize delivery routes.  
🔹 **Solar Charging:** Integrate **solar panels** for improved energy efficiency.  
🔹 **Mobile App Integration:** Provide real-time updates through a **dedicated app**.  
🔹 **Payload Capacity:** Expand the storage to **handle larger deliveries**.  


---

## 📧 Contact  
For any inquiries or support, feel free to reach out:  
- **👤 Team Members:** Ahmad Muhammad El-Sayed, Pavli Bahaa Botrus, Yousef Ali Abdallah, Amr Ashraf Fawzy, Abdelrahman Ahmed Mohamed, Ahmed Saad El-Menawy  
- **Advisors:** Dr. Omar Shalash, Eng. Mohamed El-Sayed  
- **🌍 Web Interface:** [AUCTUS](http://www.cai.aast.edu/auctus)  

