# sonar-mapping-bot
Sonar mapping bot is a manually controlled 4WD bot to map the surroundings on a lower scale using Sonar 

The repo consists of 3 Arduino codes and 1 Python code

sonar_mapping_script.py - main python script to control the bot using keys and initiate scan. The bot's position is updated after each move and the obstacle map around the bot is updated on the graph after each successful scan.

bot_ir_feedback_check.ino - Arduino code to check and calibrate the IR feedback sensors.

bot_position_feedback.ino - Arduino code to check bot's position feedback on graph in python script

sonar_mapping_main.ino - main arduino code for bot control, scanning and sending data to the python script

[Project documentation](https://drive.google.com/file/d/1zc0qHPEwykU3vaZfMe2obiM-7MfIT7Va/view?usp=sharing)

