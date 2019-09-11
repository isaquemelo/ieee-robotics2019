
# Values that will need to be calibrated
"""
4 means the sensor is 6cm from the pipeline with the robot parallel with the pipeline
"""
setpoint_for_pid_pipeline_fowler = 4
# Values that will need to be calibrated


# CALIBRATION ROUTINES

"""
find the maximum variation of the white from the meeting area, so then u can define consts for the pid from 
robot.black_line_following()

obs: 
- all this is referenced to the meeting are
- the sensor is the colorSensor in mode = 'REF-RAW'

- setpoint -> will be reference as the value of the sensor when it's 60% on the white and other 40% on the black line 
- white_value -> it is the smallest value from the sensor when it was on the white part
- max_white_var -> it is the difference biggest - smallest
"""
black_line_following = {"white_value": 435, "max_white_var": 30, "setpoint": 457}
def find_max_white_var(robot):
    colors = robot.get_sensor_data("ColorSensor")
    smallest = colors[0]
    biggest = colors[0]

    while True:
        colors = robot.get_sensor_data("ColorSensor")
        if colors[0] < smallest:
            smallest = colors[0]
        if colors[1] < smallest:
            smallest = colors[1]
        if colors[0] > biggest:
            biggest = colors[0]
        if colors[1] > biggest:
            biggest = colors[1]

        print("biggest = ", biggest, " - ", "smallest = ", smallest, " == ", biggest - smallest)

# CALIBRATION ROUTINES
