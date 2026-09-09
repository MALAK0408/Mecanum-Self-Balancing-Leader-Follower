# Mecanum & Self-Balancing Leader–Follower Robot

##  Project Overview

This project presents a Leader–Follower robotic system developed using two UGOT robots.

The first robot is a **Mecanum robot**, which acts as the **Leader**. The second robot is a **Self-Balancing robot**, which acts as the **Follower**.

The Follower uses a camera and **AprilTag detection** to identify and follow the Leader while maintaining an approximately 15 cm distance.

---

##  Robots Used

### 1. Mecanum Robot — Leader

The Mecanum robot acts as the Leader of the system.

It performs a predefined movement sequence, including:

- Forward movement
- Slow movement before turning
- Approximately 90° turns
- Waiting for the Follower to detect the AprilTag
- Slow movement after the turn
- Temporary backward movement when the Follower loses the tag for too long

---

### 2. Self-Balancing Robot — Follower

The Self-Balancing robot acts as the Follower.

It uses its camera to detect the AprilTag attached to the Leader.

When the tag is detected, the robot follows the Leader and adjusts its speed according to the estimated distance.

The target distance is approximately **15 cm**.

---

##  AprilTag Detection

An AprilTag is used as the visual reference between the two robots.

The Follower searches for the specific target tag and uses its position and apparent size to control its movement.

The system also includes a search strategy when the AprilTag is temporarily lost.

---

##  Distance Control

The Follower uses proportional control to maintain the desired distance from the Leader.

The target distance is approximately:

**15 cm**

The speed is automatically adjusted according to the detected tag size.

A small tolerance zone is also used to reduce unnecessary oscillations.

---

##  Leader–Follower Operation

The general operation of the system is:

1. The Leader starts moving forward.
2. The Follower detects the AprilTag.
3. The Follower follows the Leader.
4. The Leader slows down before turning.
5. The Leader performs a turn.
6. The Leader stops temporarily.
7. The Follower searches for the AprilTag if it is lost.
8. Once the tag is detected again, the Follower continues following.
9. The Leader resumes its trajectory.
10. If the tag remains lost for a long time, the Leader moves backward slowly to help the Follower recover the tag.

---

##  Tag Loss and Search Strategy

When the Follower loses sight of the AprilTag, it does not immediately perform a random movement.

The system uses several search phases:

- Short forward movement
- Search in the last probable direction
- Search in the opposite direction
- Short forward movement again

The direction of the first search is determined using the last known horizontal position of the AprilTag.

---

##  Technologies Used

- Python
- UGOT Robotics Platform
- OpenCV
- NumPy
- AprilTag Detection
- Computer Vision
- Mecanum Drive
- Self-Balancing Robot
- Proportional Control

---

##  Project Structure

```text
Mecanum-Self-Balancing-Leader-Follower/
│
├── README.md
│
├── code/
│   └── leader_follower.py
│
└── images/
    ├── mecanum_robot.jpg
    ├── self_balancing_robot.jpg
    └── robots.jpg
