# **Autonomous VIO-based Quadcopter**

Welcome to my Autonomous VIO-based Quadcopter project! This repository showcases the culmination of my efforts in integrating state estimation, trajectory planning, and control mechanisms for a quadcopter using Visual Inertial Odometry (VIO). The project aims to provide a seamless flight experience in a simulated environment.

## **Highlights**

- ***State Estimation***: The quadcopter is equipped to estimate its state in real-time, even amidst noisy sensor data.
- ***Trajectory Planning***: Advanced algorithms are in place to ensure the quadcopter plans its flight trajectory efficiently.
- ***Control Mechanism***: A robust control system is implemented to ensure the quadcopter adheres to the planned trajectory.
- ***Visual Inertial Odometry (VIO)***: The cornerstone of this project, VIO enables the quadcopter to function even in environments devoid of GPS.

## **Getting Started**

1. ***Clone the Repository***:

git clone https://github.com/NiceLionel/Quadrotor-Simultaneous-Localization-Planning-and-Control.git

2. ***Setup***:
Navigate to the project directory and install the necessary packages:

cd Quadrotor-Simultaneous-Localization-Planning-and-Control
python setup.py install

4. ***Run the Simulation***:
Launch the simulation and observe the quadcopter in action:

python path_to_main_script.py


## **Features**

- ***Simulator Implementation***: The simulator is designed to synthesize camera images based on the quadcopter's position, offering a lifelike flight experience.
- ***Extra Challenges***: For those seeking a deeper dive, challenges such as online mapping, monocular VIO, and VIO loop-closure have been incorporated. Dive in and explore!

## **Contributions**

Your insights and contributions are always welcome! If you have suggestions, improvements, or feedback, please fork this repository and submit a pull request. Together, we can enhance this quadcopter project!

## **License**

This project is licensed under the [MIT License](https://choosealicense.com/licenses/mit/).
