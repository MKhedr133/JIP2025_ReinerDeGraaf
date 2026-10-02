# Autonomous Radiation Detection Robot

ROS 2 mobile robot developed for a TU Delft multidisciplinary healthcare
robotics project with Reinier de Graaf hospital.

The system combines **radiation sensing, autonomous source seeking, SLAM,
localization, sensor fusion, and navigation** to investigate how a mobile
robot could support radiation contamination tasks while reducing unnecessary
human exposure.

> University research prototype. This system is not a certified medical device.

---

## Project overview

Radiation contamination work can require staff to enter areas where exposure
should be kept as low as reasonably possible.

This project investigates a mobile robotic approach in which a robot can:

- measure radiation using a physical scintillation detector
- move through an indoor environment
- build and use a map
- estimate its position
- autonomously search for a radiation source
- provide manual control when required

The system was developed using an **iRobot Create 3** mobile platform and
**ROS 2 Humble**.

---

## System architecture

```mermaid
flowchart LR
    A[Berthold LB-124] --> B[ROS 2 radiation driver]
    B --> C[CPS filtering and logging]

    D[RPLIDAR] --> E[SLAM / Mapping]
    F[Odometry + IMU] --> G[EKF sensor fusion]

    E --> H[Map]
    G --> I[Robot pose]

    H --> J[Localization / Nav2]
    I --> J

    C --> K[Autonomous radiation search]
    I --> K

    K --> L[Velocity commands]
    J --> L

    M[Keyboard teleoperation] --> L

    L --> N[iRobot Create 3]
