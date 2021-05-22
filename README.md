# sonar-mapping-bot
Sonar mapping bot is a manually controlled 4WD bot to map the surroundings on a lower scale using Sonar 

The basic idea is to implement manual mapping of an arena/room using sonar mounted on a differential drive bot. So, using Bluetooth communication, the bot’s movement is controlled across the arena and the ultrasonic sensor picks up distance of objects around it (walls) and sends it to PC controller (python program) which further manipulates the data received and displays the raw mapping on the graph. After the entire mapping is complete, the python program processes and filters the raw data to get a closed map of the arena.

Find project documentation [here](https://drive.google.com/file/d/1zc0qHPEwykU3vaZfMe2obiM-7MfIT7Va/view?usp=sharing)

## About the project
### Chassis assembly and motors
A 11x18cm Hylam board was used as base and 4 motors was stuck to the base. Wires was soldered to the motors and 2 motors on each side were connected in parallel. Other end of the wires was soldered to male connector pins. Holes were made at centre of chassis to bring the wires overboard.

### Bot position feedback
To get position of bot, wheel rotation count is required. IR sensors was used and a module was fabricated for the same considering the fact that the rotation speed was low and we get the spoke count without much error. Hence 5mm IR transmitter and receiver were stuck to a piece of perf board further stuck to the bot, close to the wheels. The analog signal obtained was calibrated to convert into digital counts in the Arduino sketch. This allows using the bot in varied lighting conditions, calibrating in an environment considering the external noise build up.

MPU6050 module is used to get yaw of bot at particular instants. The MPU library example code was relied to initiate DMP and calculate yaw from raw data received.

### Ultrasonic sensor mounted on Servo
At required commands, the servo sweeps 180 degrees at 2 degrees each to get distance values at different angles. The servo is attached and detached at start and end of the particular commands to avoid errors. At each iteration, the ultrasonic sensor takes distance measurements and the servo proceeds.

HCSR04 module is mounted on servo to read distance values at particular angles and the distance values is sent to PC via Bluetooth. The distance values are calibrated to conditions.

### Python program
Required libraries like matplotlib for plotting graph, serial and time for Bluetooth communication, numpy for coordinate transforms, math for operations, are used. Entire mapping algorithm occurs in the python code. Based on input from user, the python code sends characters to bot for control and receives data from the bot at the end of each character sent. The received data is processed according to formulae and converted to required coordinates with respect to a common origin frame and stored for plotting on graph and later computations.

### Coordinate transform
This idea was from forward kinematics and concept of homogenous transforms in mechanics, where the coordinate frame is rotated and translated using matrix homogenous transform. Same approach has been implemented here to convert the distance coordinates with respect to bot into graph with respect to origin.
The required matrices are initialised and operated to get the final coordinates in the python script.

## Contents of repository
The repo consists of 3 Arduino codes and 1 Python code
- sonar_mapping_script.py - main python script to control the bot using keys and initiate scan. The bot's position is updated after each move and the obstacle map around the bot is updated on the graph after each successful scan.
- bot_ir_feedback_check.ino - Arduino code to check and calibrate the IR feedback sensors.
- bot_position_feedback.ino - Arduino code to check bot's position feedback on graph in python script
- sonar_mapping_main.ino - main arduino code for bot control, scanning and sending data to the python script




