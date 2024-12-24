**Course Project and Assignments for CSE633: Introduction to Robotics at IIIT Delhi**

Assignment-1: Implement a PD Controller on a Point Mass to simulate the oscillatory motion of a pendulum.

Assignment-2: Implement a Pure Pursuit Controller on a Unicycle Robot to track the unit circle centered at origin.

Assignment-3: Implement a Feedback Controller in ROS and Gazebo.

Hardware Project: Build a Line Follower capable of tracking linear, sinusoidal and circular paths.

# Project Desc

# Line Following Mobile Robot Design  

## Course Project: Robotics  

**Group Members**  
- Arnav Singh (2021019)  
- Yash Khatri (2020267)  

---

## Project Overview  

This project involved designing and developing a line-following mobile robot capable of navigating predefined test tracks. The robot was built using a DIY robotics kit, integrating IR sensors, microcontrollers, and an L293D motor shield to detect and follow black tape curves on A0-sized tracks. The focus was on achieving flexible and adaptive traversal behavior.  

---

## Objectives  

1. **Assembling the Robot**  
   - Utilize a DIY robotics kit.  
   - Attach motors to the chassis with tape and screws.  
   - Connect the power sources, unicycle wheel, and castor wheels.  
   - Integrate the L293D motor shield with the UNO R3 microcontroller.  
   - Link IR sensors to the microcontroller using F-F wires.  

2. **Coding the Feedback Controller**  
   - Develop an algorithm to detect and follow track curves regardless of the robot's initial orientation.  

3. **Tuning the Sensors**  
   - Calibrate the IR sensors using a screwdriver to accurately differentiate between black and white surfaces.  

---

## Components Used  

- **Chassis**  
- **IR Sensors** (3)  
- **L293D Motor Shield**  
- **UNO R3 Microcontroller**  
- **DC Motors** (4)  
- **Wheels** (unicycle and castor)  
- **Power Sources**  
  - Xiaomi 20,000 mAh power bank (via USB)  
  - 4 Duracell AA batteries (via power jack)  
- Miscellaneous: Double-sided tape, L-Clamps, screws, nuts.  

---

## How It Works  

The robot uses three IR sensors (left, center, and right) to detect its path by distinguishing between black and white surfaces. These sensors provide binary input to the UNO R3 microcontroller through the L293D motor shield, where:  
- **1** = Black surface (track)  
- **0** = White surface  

The microcontroller processes the data using a custom loop algorithm and directs the four DC motors accordingly, enabling the robot's movement.  

Power is supplied via two sources:  
1. **Power bank**: Adds stability by shifting the center of mass towards the back, improving smooth motion.  
2. **AA batteries**: Supports the motor shield.  

The algorithm includes an initialization function suggested by the course TA, enabling the robot to move forward until it detects a black path.  

---

## Results  

The robot performed consistently across all tracks, demonstrating notable success on the third track, where its performance improved iteratively under unconventional starting conditions. Despite constraints on implementing a PID controller due to limited lab access later in the semester, the bang-bang controller achieved commendable results.  

This project provided valuable insights into robot dynamics and modeling, bridging theory and practical application effectively.  
