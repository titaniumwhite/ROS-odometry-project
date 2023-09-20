# Project for Robotics course #
### @Politecnico di Milano, Academic Year 2021-2022 ###
The aim of this project was to develop a comprehensive odometry and transformation frame (TF) system for a skid-steering robot. For specific information read the instructions.

#### Project Components: ####
- Odometry Computation: The project aimed to compute odometry information for a skid-steering robot, estimating angular and linear velocities of the wheels to calculate the robot's position and orientation over time.
- Transformation Frame (TF): Implementation of a TF from /odom to /base_link was crucial for monitoring the robot's position and orientation, allowing comparisons with manufacturer-provided odometry data.
- Integration Methods: Users could choose between two integration methods, Euler and Runge-Kutta, for odometry calculations, providing flexibility for different scenarios.
- Reset Services: The project included services to reset odometry, either to the origin (0, 0) or specific provided values (x, y, theta).
- Custom Message: A custom message structure, "CustomOdometry," facilitated the exchange of odometry data between system components, including the selected integration method.
- Noise Estimation: The project involved fine-tuning parameters related to the skid-steering robot, such as wheel velocities and apparent baseline, using experimental methods to improve odometry accuracy.
- Visualization and Verification: Visualization tools like RVIZ were used for real-time monitoring and verification of odometry data and the TF, ensuring accuracy and reliability.
