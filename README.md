Autonomous VIO-based Quadcopter
Welcome to the Autonomous VIO-based Quadcopter project! This repository houses an advanced simulation and control system for a quadcopter using Visual Inertial Odometry (VIO). Dive in to explore how I integrated state estimation, trajectory planning, and control mechanisms to achieve a seamless flight experience in a simulated environment.

Simulation Image

(Replace path_to_simulation_image.png with the actual path to your simulation image if available)

Overview
State Estimation: My quadcopter can estimate its state in real-time, even with noisy sensor data.
Trajectory Planning: I've developed algorithms that allow the quadcopter to plan its flight trajectory efficiently.
Control Mechanism: My control system ensures the quadcopter follows the planned trajectory with precision.
Visual Inertial Odometry (VIO): The heart of this project, VIO allows the quadcopter to operate even in GPS-denied environments.
Getting Started
Clone the Repository:

bash
Copy code
git clone https://github.com/NiceLionel/Quadrotor-Simultaneous-Localization-Planning-and-Control.git
Setup:
Navigate to the project directory and install the required packages:

cd Quadrotor-Simultaneous-Localization-Planning-and-Control
python setup.py install
Run the Simulation:
Kickstart the simulation and watch the quadcopter in action!

bash
Copy code
python path_to_main_script.py
(Replace path_to_main_script.py with the actual path to your main script if available)

Features
Simulator Implementation: My simulator synthesizes camera images based on the quadcopter's position, offering a realistic flight experience.
Extra Challenges: For the enthusiasts out there, I've included challenges like online mapping, monocular VIO, and VIO loop-closure. Check them out!
Contributing
I appreciate contributions! If you have suggestions or improvements, feel free to fork this repository and submit a pull request. Let's make this quadcopter even better together!

License
This project is licensed under the MIT License.
