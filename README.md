# Remote-Controlled-Car
For this project I chose to create a remote controlled car with collision detection features. This project was inspired by electric skateboards as they both used similar design components. This project will be converting a RC (Remote Controlled) car, by removing the remote and receiver aspects and replacing them with microcontrollers. These microcontrollers will take the jobs of the receiver as a slave to the transmitting master remote.

The main method of communication was using bluetooth between the master and slave. The slave microcontroller handled the calculations to detect objects and corrected the steering and throttle to avoid the obstacle. The master microcontroller collected input from a joystick and broadcasted the information via bluetooth to the slave. However, in my case I had used USART instead of the planned Bluetooth communication.

Video Demonstration: https://youtu.be/zplhUXiXwTA
