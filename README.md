# Neetram
Our mission is to create autonomous aerial solutions that can assist in:

🌪️ Post-disaster surveillance (floods, earthquakes, landslides, fire zones)

📡 Search & rescue missions with onboard sensors & custom antenna systems

🚑 Medical & supply delivery in inaccessible zones

🛰️ Real-time data collection & transmission, even in disrupted networks

🛡️ Self-protection & defense mechanisms to ensure mission reliability in harsh conditions

This repository hosts everything from CAD designs to ROS2 simulation environments, enabling both hardware prototyping and virtual validation.

Repository Structure
📦 Neetram

 ┣ 📁 CAD Designs        # Drone CAD files (SolidWorks/ANSYS)
 
 ┣ 📁 Simulations # Structural & aerodynamic simulations
 
 ┣ 📁 ROS2_Code         # ROS2 nodes for navigation, control, sensors
 
 ┣ 📁 Gazebo_Worlds     # Gazebo simulation environments
 
 ┣ 📁 UAV_Algorithms    # Path planning, obstacle avoidance, swarm logic
 
 ┣ 📁 Real-Time-Drone   #The parts of the drone which are being/already built
 
 ┣ 📁 Antenna_Designs   # Custom antenna CAD + testing data
 
 ┣ 📁 ML_Model_Results  # YOLO and OpenCV outcomes
 
 ┗ README.md            # You are here 🚀

🛠️ Tech Stack

Hardware: Custom UAV frame, onboard sensors (GPS, IMU, LiDAR, Camera, Gas sensors, Thermal imaging, RF modules)

Custom Antenna:
Designed for extended range in disaster-hit low-connectivity zones

Optimized for signal penetration through debris and urban clutter

Integrated with multi-frequency bands for redundancy

Defense Mechanisms:

🛡️ Fail-safe landing system during signal loss

🔋 Battery redundancy for extended mission reliability

📶 Anti-jamming protocols for communication resilience

🌐 Collision avoidance sensors for safe navigation

Software & Tools:

ROS2 Humble

 – Autonomy & control
 
Gazebo

 – Simulation & testing
 
ANSYS
 – Aerodynamic & structural analysis
 
PX4 / Ardupilot

 – Flight stack integration
 
[Python / C++] – Algorithms & control nodes

🚀 Features

🔍 Autonomous Navigation

Complete open-source flight control algorithm (Python) with integrated PID tuning for stable and precise UAV maneuvers.

🧠 Onboard AI ( ML Object Detection)

Jetson/Raspberry Pi powered real-time object detection for victim recognition, hazard detection, and intelligent mission planning.

🚑 Survival Kit Delivery

Payload delivery mechanism for first-aid kits, communication devices, and essential supplies in inaccessible disaster zones.

🎥 Long-Range Video & Data Transmission

Uses GStreamer + MQTT for low-latency HD video and sensor telemetry streaming over long distances.

📡 Custom Triple-Band Patch Antenna

Supports 5.8 GHz, 2.4 GHz, and 900 MHz for redundancy and range, ensuring communication even in disrupted networks.

⚙️ PID Tuning & IMU Data Integrity

Fine-tuned PID controllers with sensor fusion (IMU + GPS), combined with data integrity checks and encryption.

🛡️ Anti-Jamming & FHSS (Frequency Hopping Spread Spectrum)

Defensive communication strategy to counter signal interference, with compliance-based declination of offensive jamming use.

🔐 Transmission Encryption & Security Firewall

All communication channels encrypted; dedicated firewall layer shields flight systems from unauthorized intrusion.

🔑 Flight Control Password Protection

Adds secure authentication layers before drone takeoff and system configuration, ensuring only authorized operators can access controls.

🗺️ Custom Simulation Worlds

Disaster-specific Gazebo environments — flood zones, fire scenarios, collapsed structures — for training and validation.


📸 Visuals
Drone CAD (ANSYS/3D Model)
![97d64978-04e4-4525-a957-2996860e7fa1](https://github.com/user-attachments/assets/ea72b3b3-afea-4ec4-9be8-506ff525c3b8)

Gazebo Simulation

![746edade-ba8a-4d0c-b988-6a8fd66bfb27](https://github.com/user-attachments/assets/074cf582-6042-478a-abbd-df80fc0b8cbd)

Antenna(In progress)

![d254aaba-9140-4178-a9ef-ec0109d70a65](https://github.com/user-attachments/assets/6a6ea1f8-dd78-41b9-a11a-ab8b9fc5d6e2)

Ground Station View

![fb0db4fb-a4b2-4d98-b15e-09548827703f](https://github.com/user-attachments/assets/36d0dcf9-dd9a-4427-9d9c-b23f293dc86c)


📌 MVP (Minimum Viable Product): [MVP]()
📱 Mobile/Web App for Command & Control: [App](https://neetram.onhercules.app/)

👩‍🚀 Team Neetram

Name	             -         Role       	 -     Focus Area

Deep Batyabal	     -     Project Lead    -    UAV Systems

Ritabhasa Chowdhury	 -   Software Engineer	-   ROS2 Integration

Sagnik Chakraborty  -  Simulation Engineer	-  ANSYS, CAD

Prasunita De	   -      Controls Engineer	-    Flight Algorithms

Aditi Rai	       -       AI/ML Engineer	  -   Vision & Detection

Sagnik Adhikary	 -     Hardware Engineer	-   Sensors, Antenna & Payloads

🌟 Impact

By combining cutting-edge drone technology with custom communication systems and self-defense mechanisms, Neetrum aims to:

Reduce response time in disasters

Provide real-time situational awareness

Save lives & resources through smart deployment
