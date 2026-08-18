### Hardware Platform: SVAN M2

Our project focuses on developing a novel controller to be deployed directly on the onboard computer of the **SVAN M2**, India's first commercial quadruped robot, developed by **[xTerra Robotics](https://xterrarobotics.com/svan-m2/)**.

Key features and technical specifications of the SVAN M2 platform relevant to our development include:
- **Onboard Computing (Target for Custom Controller):** An embedded ROS/ROS2-compatible computational framework capable of handling high-speed control loops. This serves as the primary deployment environment for our newly developed control architecture and navigation pipelines.
- **Agile Locomotion:** A 12-DOF leg system powered by proprietary high-torque QDD (Quasi-Direct Drive) actuators, allowing our controller to execute dynamic movements, stair climbing, and stable traversal across unstructured terrains.
- **Perception & State Estimation:** Integrated Inertial Measurement Units (IMUs), joint encoders, and high-definition vision capabilities that provide the real-time proprioceptive feedback necessary for our custom control loops.
- **Payload & Endurance:** Supports a 5 kg payload capacity and features a 2-4 hour battery life, which allows for extended real-world testing and tuning of our algorithms.
