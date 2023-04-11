
# STM32PILOT

STM32PILOT is a project intended to explore the STM32 world while creating a basic fixed-wing autopilot. The project mainly uses X-Plane 11 simulator, although some sections have already been implemented using real hardware, such as MPU6050.

## Demo
Check out some demos of the project in action:

- Take-Off using remote controler:
    ![Take-Off](https://github.com/udi741/STM32PILOT/tree/main/git-media/TakeOff.mp4)
- Attitude control - Roll Hold (pitch and throttle controlled by the user):
    ![Roll Hold](https://github.com/udi741/STM32PILOT/tree/main/git-media/RollHold.mp4)
- Advanced - Fly toward list of Waypoints autonomously:
    ![Fly to waypoints](https://github.com/udi741/STM32PILOT/tree/main/git-media/FlyToWaypoints.mp4)


# Requirements

## Software
- Keil uvision 5 or any other IDE that uses STM32 libraries to compile the code
- X-Plane 11 - Demo version [X-Plane download site](https://www.x-plane.com/desktop/try-it/older/)
- Python 3 to communicate between X-Plane and STM32PILOT


## Hardware
- STM32F103C8T6 "maple mini". (in order to use another version, changes to the code are required)
- USB to TTL adapter - FT232RL (UART) to communicate with the X-Plane simulator
- Optional: any logic analyzer, helped me alot to figure what is wrong in ruff times
- FLYSKY FS-I6X remote controller
- FLYSKY FS-iA10B receiver
## Installation
To install and set up the project:

1. Clone the repository: git clone https://github.com/udi741/STM32PILOT.git
2. Install the required dependencies using pip: pip install -r py/requirements.txt
3. Open the project in Keil uvision 5 or your preferred IDE.
4. Compile and upload the code.
## Running the Project
To run the project:

1. Connect the STM32 board to your computer using the USB to TTL adapter.
2. Connect the RC receiver to the STM32 board.
3. Open X-Plane
4. Run STM32PILOT.py
5. Start a new flight
6. Enjoy :)