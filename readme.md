
# STM32PILOT

STM32PILOT is a project intended to explore the STM32 world while creating a basic fixed-wing autopilot. The project mainly uses X-Plane 11 simulator, although some sections have already been implemented using real hardware, such as MPU6050.

## Demo
Check out some demos of the project in action:

- Take-Off using remote controler:<br>
    ![Take-Off](https://github.com/udi741/STM32PILOT/blob/main/git-media/TakeOff.gif)
- Attitude control - Roll Hold (pitch and throttle controlled by the user):<br>
    ![Roll Hold](https://github.com/udi741/STM32PILOT/blob/main/git-media/RollHold.gif)
- Advanced - Fly toward list of Waypoints autonomously:<br>
    ![Fly to waypoints](https://github.com/udi741/STM32PILOT/blob/main/git-media/FlyToWaypoints.gif)<br>
    <img src="https://github.com/udi741/STM32PILOT/blob/main/git-media/Roll_AileronCmd.png" title="Roll and Aileron command" width="600" height="450"></img><br>
    <img src="https://github.com/udi741/STM32PILOT/blob/main/git-media/DistanceToWP.png" title="Distance to next waypoin" width="600" height="450"></img><br>
    <img src="https://github.com/udi741/STM32PILOT/blob/main/git-media/FlightPath.png" title="Flight path" width="600" height="450"></img><br>


# Requirements

## Software
- An IDE that uses the appropriate STM32 development environment and libraries for the specific STM32 microcontroller used in this project.<br>Keil uVision 5 was used for this project.
- X-Plane 11 - Demo version [X-Plane download site](https://www.x-plane.com/desktop/try-it/older/)
- Python 3 to communicate between X-Plane and STM32PILOT

## Hardware
- STM32F103C8T6 "maple mini". (in order to use another version, changes to the code are required)
- USB to TTL adapter - FT232RL (UART) to communicate with the X-Plane simulator
- FLYSKY FS-I6X remote controller
- FLYSKY FS-iA10B receiver
- Optional: any logic analyzer, helped me alot to figure what is wrong in rough times

# Installation
To install and set up the project:

1. Clone the repository: ```git clone https://github.com/udi741/STM32PILOT.git```
2. Install the required dependencies using pip: pip install -r py/requirements.txt
3. Open the project in Keil uvision 5 or your preferred IDE.
4. Compile and upload the code.

# Running the Project
To run the project:

1. Connect the STM32 board to your computer using the USB to TTL adapter.
2. Connect the RC receiver to the STM32 board.
3. Open X-Plane
4. Run STM32PILOT.py UART_COM [UDP_IP UDP_PORT]
5. Start a new flight
6. Enjoy :)

# FAQ

##### How to switch between fligt modes
Note: the answer is related to the RC in the Hardware section.
	when mentioning the RC switches, states term will be used (state 1-3, when 1 is pointing up, 2 is one step down, and 3 is two steps down)
	when mentioning the RC Knobs, pwm value will be used (1000-2000, where 1000 is knob rotation to the most left, 2000 most right)
1. Disco mode - SWA (state 1)
2. Roll Hold - SWA (state 2), SWC (state 1), VARB (1000-1100)
3. Pitch Hold - SWA (state 2), SWC (state 2), VARB (1101-1500)
4. Attitude Hold (keeps current pitch and roll) - SWA (state 2), SWC (state 2), VARB (1501-2000)
5. Fly toward Waypoints - SWA (state 2)
6. Alt Hold - SWA (state 3)
