#!/usr/bin/env python3

def undefined_dealing(robot, color_sensor):
    print("chamou a undefined dealing")
    limiar = 12
    sensor_color = color_sensor
    speed = 400
    if sensor_color[0] == "Undefined" or sensor_color[1] == "Undefined":
        if sensor_color[0] == "Undefined":
            #robot.move_timed(how_long=0.6, direction="back")
            # SUBSTITUI O MOVETIMED DE CIMA
            while True:
                search = robot.sensor_data("ColorSensor")
                if "Undefined" not in search:
                    robot.stop_motors()
                    break
                else:
                    robot.motors.left.run_forever(speed_sp=-speed)
                    robot.motors.right.run_forever(speed_sp=-speed)
            # SUBSTITUI O MOVETIMED DE CIMA
            # robot.rotate(30, axis="diferente")
            # SUBSTITUI A ROTATE DE CIMA
            robot.gyroscope_sensor.mode = 'GYRO-RATE'
            robot.gyroscope_sensor.mode = 'GYRO-ANG'
            while True:
                if robot.sensor_data('GyroSensor') >= limiar:
                    robot.stop_motors()
                    break
                else:
                    robot.motors.right.run_forever(speed_sp=-speed)
            #time.sleep(5)
            # deal_with_rotation_from_undefined_dealing(robot, left_out=True)
            # SUBSTITUI A ROTATE DE CIMA
        elif sensor_color[1] == "Undefined":
            #robot.move_timed(how_long=0.6, direction="back")
            # SUBSTITUI O MOVETIMED DE CIMA
            while True:
                search = robot.sensor_data("ColorSensor")
                if "Undefined" not in search:
                    robot.stop_motors()
                    break
                else:
                    robot.motors.left.run_forever(speed_sp=-speed)
                    robot.motors.right.run_forever(speed_sp=-speed)
            # SUBSTITUI O MOVETIMED DE CIMA
            # robot.rotate(-30, axis="diferente")
            robot.gyroscope_sensor.mode = 'GYRO-RATE'
            robot.gyroscope_sensor.mode = 'GYRO-ANG'
            while True:
                if robot.sensor_data('GyroSensor') <= -limiar:
                    robot.stop_motors()
                    break
                else:
                    robot.motors.left.run_forever(speed_sp=-speed)
            # time.sleep(5)
            # SUBSTITUI A ROTATE DE CIMA
            # deal_with_rotation_from_undefined_dealing(robot, right_out=True)
            # SUBSTITUI A ROTATE DE CIMA
    print("retornou da undegined dealing")