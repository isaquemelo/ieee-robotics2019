#!/usr/bin/env python3

import ev3dev.ev3 as ev3
import math
from datetime import datetime, timedelta
import time
import paho.mqtt.client as mqtt
from struct import *
from simple_pid import PID
from assets.classes.robot import Robot
from rescue import rescue, bounding_box
import json
from assets.handlers.button import ButtonApproach
from assets.handlers.undefined_dealing import undefined_dealing
from assets.handlers.server_config import Server

DEFAULT_SPEED = 350





last_same_color = [None, ""]
color = 0
rect_check = False

# 15.6 0 4.8
pid = PID(15.6, 0, 4.8, setpoint=-4)
ok = True

def color_realignment(robot, color_sensor_data, infrared_sensor, move_forward=True, speed=DEFAULT_SPEED):
    deu_re = False
    limiar = 20 # 15
    li = 7
    limiar_time = 0.6
    limiar_speed = speed
    robot.update()
    global last_same_color, color, rect_check
    reverse = False

    search = color_sensor_data
    control = pid(robot.infrared_sensors[1] - robot.infrared_sensors[0])

    # print("SEARCH = ({}, {})".format(search[0], search[1]))

    if robot.in_rect:
        if (search[0] != robot.rect_color) or (search[1] != robot.rect_color):
            robot.rect_color = "Undefined"
            robot.in_rect = False
            return None

    n_speed = 600
    if control > 400:
        control = 400
    if control < -400:
        control = -400


    # if robot.reverse_path is None or robot.has_doll:
    #     n_speed = 600
    #     if control > 400:
    #         control = 400
    #     if control < -400:
    #         control = -400
    # else:
    #     n_speed = 280
    #
    #     if control > 60:
    #         control = 60
    #     if control < -60:
    #         control = -60

    search = robot.sensor_data("ColorSensor")

    if search[0] == search[1]:
        # print("[0] == [1]")
        if robot.sensor_data("ColorSensor")[0] != "White" and robot.sensor_data("ColorSensor")[1] != "White":
            # dentro do quadrado
            robot.motors.left.run_forever(speed_sp=speed)
            robot.motors.right.run_forever(speed_sp=speed)
        else:
            robot.motors.left.run_forever(speed_sp=n_speed + control)
            robot.motors.right.run_forever(speed_sp=n_speed - control)

        last_same_color = search

        if search[1] not in ["White", "Undefined", "Brown", "Black"]:
            color += 1
            print("COLOR = {}".format(color))

        if color > 8:
            ev3.Sound.beep()

            # print("COLOR = {}".format(color)).
            # print("SEARCH = {}".format(search))

            search = robot.sensor_data("ColorSensor")
            if search[0] in ["White", "Undefined"] or search[1] in ["White", "Undefined"]:
                color = 0
                robot.stop_motors()

                while True:
                    search = robot.sensor_data("ColorSensor")
                    if search[0] in ["White", "Undefined"] and search[1] in ["White", "Undefined"]:
                        robot.motors.left.run_forever(speed_sp=-500)
                        robot.motors.right.run_forever(speed_sp=-500)
                    else:
                        robot.stop_motors()
                        break

                # Pronto para confirmar onSquare. Aguarda ficar dentro do quadrado para retornar
                while True:
                    search = robot.sensor_data("ColorSensor")
                    if search[1] in ["Blue", "Green", "Red"] or search[0] in ["Blue", "Green", "Red"]:
                        robot.stop_motors()
                        break
                    else:
                        robot.motors.right.run_forever(speed_sp=-500)
                        robot.motors.left.run_forever(speed_sp=-500)

                while True:
                    search = robot.sensor_data("ColorSensor")
                    if search[0] in ["Blue", "Green", "Red"]:
                        robot.stop_motors()
                        break
                    else:
                        robot.motors.left.run_forever(speed_sp=-500)

                while True:
                    search = robot.sensor_data("ColorSensor")
                    if search[1] in ["Blue", "Green", "Red"]:
                        robot.stop_motors()
                        break
                    else:
                        robot.motors.right.run_forever(speed_sp=-500)

                robot.move_timed(how_long=0.2, direction="back")
                robot.in_rect = True

                if robot.sensor_data("ColorSensor")[0] not in ["White", "Undefined"]:
                    robot.rect_color = robot.sensor_data("ColorSensor")[0]
                elif robot.sensor_data("ColorSensor")[1] not in ["White", "Undefined"]:
                    robot.rect_color = robot.sensor_data("ColorSensor")[1]

                # print("I'm on a square", robot.rect_color)
                rect_check = True
                # print("RETORNOU ON SQUARE COM  RECT = {}".format(robot.rect_color))
                return "On square"

    elif search[0] == "Undefined" and search[1] != "Undefined" or search[1] == "Undefined" and search[0] != "Undefined":
        # if last_same_color == ["White", "White"]:
        #     print("Running undefined dealing...")
        #     undefined_dealing(robot, search)
        #
        # else:
        #     print("Undefine Dealing counting..")
        #     robot.undefined_counter += 1
        #
        # if robot.undefined_counter > 5:
        #     print("Undefined Dealing executed..")
        #     robot.undefined_counter = 0
        #     undefined_dealing(robot, search)
        robot.stop_motors()
        undefined_dealing(robot, search)


    # A ULTIMA CONDICIONAL SERVE PARA EVITAR QUE O ROBO TENTE DAR COLOR REALIGMENT NA SAIDA DA BOUNDING BOX
    elif search[0] == "White" and search[1] != "White" and search[1] not in ["Undefined", "Brown", "Black"] and datetime.now() >= robot.time_desabilita_o_realinhamento_da_cor and robot.ta_na_ranpa is False:
        # print("[0] == White [1] != White")
        if last_same_color[0] == "White" and last_same_color[1] == "White":
            reverse = True
        else:
            reverse = False

        robot.motors.left.stop()
        robot.motors.right.stop()
        ja_zerou = False
        starting_angle = robot.sensor_data("GyroSensor")
        actual_angle = robot.sensor_data("GyroSensor")

        while True:
            if ja_zerou is False:
                ja_zerou = True
                robot.gyroscope_sensor.mode = 'GYRO-RATE'
                robot.gyroscope_sensor.mode = 'GYRO-ANG'
            if reverse:
                robot.motors.left.run_forever(speed_sp=speed)

            else:
                robot.motors.right.run_forever(speed_sp=speed)

            # CASO O REALIGMENT SE TORNE MUITO ALTO
            if robot.sensor_data("GyroSensor") < -limiar:
                deu_re = True
                ev3.Sound.beep()
                ev3.Sound.beep()
                grau = robot.sensor_data("GyroSensor")
                while grau < 0 + li:
                    # print("CASO 1")
                    grau = robot.sensor_data("GyroSensor")
                    robot.motors.right.run_forever(speed_sp=-speed)
                robot.stop_motors()
                robot.move_timed(how_long=limiar_time, direction="back", speed=limiar_speed)

            elif robot.sensor_data("GyroSensor") > limiar:
                deu_re = True
                ev3.Sound.beep()
                ev3.Sound.beep()
                grau = robot.sensor_data("GyroSensor")
                while grau > 0 - li:
                    # print("CASO 2")
                    grau = robot.sensor_data("GyroSensor")
                    robot.motors.left.run_forever(speed_sp=-speed)
                robot.stop_motors()
                robot.move_timed(how_long=limiar_time, direction="back", speed=limiar_speed)
            # CASO O REALIGMENT SE TORNE MUITO ALTO

            search = robot.sensor_data("ColorSensor")
            if search[0] == search[1]:
                break

        time.sleep(0.15)
        if deu_re is False:
            robot.move_timed(direction="back")
        # ev3.Sound.speak("Robot is moving back...").wait()
        robot.realigment_counter += 1

        # if robot.realigment_counter > 7:
        #     ev3.Sound.speak("Robot has exceed correction numbers...").wait()
        #     robot.realigment_counter = 0
        #     robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.left.stop()
        else:
            robot.motors.right.stop()

    # A ULTIMA CONDICIONAL SERVE PARA EVITAR QUE O ROBO TENTE DAR COLOR REALIGMENT NA SAID DA BOUNDING BOX
    elif search[0] != "White" and search[1] == "White" and search[0] not in ["Undefined", "Brown", "Black"] and datetime.now() >= robot.time_desabilita_o_realinhamento_da_cor and robot.ta_na_ranpa is False:
        # print("[0] != White and [1] == White")

        if last_same_color[0] == "White" and last_same_color[1] == "White":
            reverse = True
        else:
            reverse = False

        robot.motors.left.stop()
        robot.motors.right.stop()
        ja_zerou = False
        while True:
            if ja_zerou is False:
                ja_zerou = True
                robot.gyroscope_sensor.mode = 'GYRO-RATE'
                robot.gyroscope_sensor.mode = 'GYRO-ANG'

            if reverse:
                robot.motors.right.run_forever(speed_sp=speed)
            else:
                robot.motors.left.run_forever(speed_sp=speed)

                # CASO O REALIGMENT SE TORNE MUITO ALTO
                if robot.sensor_data("GyroSensor") < -limiar:
                    deu_re = True
                    ev3.Sound.beep()
                    ev3.Sound.beep()
                    grau = robot.sensor_data("GyroSensor")
                    while grau < 0 + li:
                        #print("CASO 3")
                        grau = robot.sensor_data("GyroSensor")
                        robot.motors.right.run_forever(speed_sp=-speed)
                    robot.stop_motors()
                    robot.move_timed(how_long=limiar_time, direction="back", speed=limiar_speed)

                elif robot.sensor_data("GyroSensor") > limiar:
                    deu_re = True
                    ev3.Sound.beep()
                    ev3.Sound.beep()
                    grau = robot.sensor_data("GyroSensor")
                    while grau > 0 - li:
                        #print("CASO 4")
                        grau = robot.sensor_data("GyroSensor")
                        robot.motors.left.run_forever(speed_sp=-speed)
                    robot.stop_motors()
                    robot.move_timed(how_long=limiar_time, direction="back", speed=limiar_speed)
                # CASO O REALIGMENT SE TORNE MUITO ALTO

            search = robot.sensor_data("ColorSensor")
            if search[0] == search[1]:
                break

        time.sleep(0.15)
        if deu_re is False:
            robot.move_timed(direction="back")
        #ev3.Sound.speak("Robot is moving back...").wait()
        robot.realigment_counter += 1

        if robot.realigment_counter > 7:
            ev3.Sound.beep().wait()
            ev3.Sound.beep().wait()
            robot.realigment_counter = 0
            robot.move_timed(direction="forward", how_long=0.6)

        if reverse:
            robot.motors.right.stop()
        else:
            robot.motors.left.stop()
    else:
        #print("else")
        if robot.sensor_data("ColorSensor")[0] != "White" and robot.sensor_data("ColorSensor")[1] != "White":
            robot.motors.left.run_forever(speed_sp=speed)
            robot.motors.right.run_forever(speed_sp=speed)
        else:
            robot.motors.left.run_forever(speed_sp=n_speed + control)
            robot.motors.right.run_forever(speed_sp=n_speed - control)


def return_last_color(robot, square_color, last_choice):
    robot.rotate(-180)
    while True:
        robot.update()
        search = robot.sensor_data("ColorSensor")
        result = color_realignment(robot, search, robot.infrared_sensors)

        if result == "On square" and robot.rect_color == square_color:
            robot.in_rect = True
            break

    #print("\n\nEntregue\n\n")


robot = Robot()
server = Server()

# client = mqtt.Client()
# client.connect("10.42.0.43", 1883, 60)


def on_message(client, userdata, message):
    carga = unpack("iid", message.payload)
    robot.infrared_sensors = carga[:2]
    # print("Received message:", carga[:2], time.time() - float(carga[2]))


def on_connect(client, userdata, flags, rc):
    #print("The robots are connected with result code", str(rc))
    client.subscribe("topic/sensors")


server.client.on_connect = on_connect
server.client.on_message = on_message
server.client.loop_start()

last_same_color2 = None


def main():
    try:
        #robot.learned_colors = {'Green': 'right', 'Red': 'forward', 'Blue': 'left'}
        #robot.learned_colors = {}
        being_learned = "Undefined"
        learning_dic = {}
        im_learning = False
        result = None
        undefined_counter = 0

        while True:

            # coloquei aqui pra ver se ajudava a identificar o boneco mais rapido
            if robot.sensor_data("Ultrasonic") < 17 and not robot.has_doll:
                # robot.stop_motors()
                rescue(robot)
                # for i in range(2):
                #     print('IDENTIFICOU O BONECO')
                #     ev3.Sound.beep().wait()
                # time.sleep(2)

            print("robot.learned_colors = {}".format(robot.learned_colors))
            print("ro.ta_na_rampa = {}". format(robot.ta_na_ranpa))

            robot.update()
            if robot.bounding_box:
                robot.done_learning = True
                # print("ENTRANDO no bounding box..")
                # print("valoes na fila de igentificassao de fim de pista: {}".format(robot.fila_para_registro_do_fim))
                ev3.Sound.beep()
                ev3.Sound.beep()
                bounding_box(robot)
                continue

            search = robot.sensor_data("ColorSensor")
            result = color_realignment(robot, search, robot.infrared_sensors)
            # print("COLOR_REALIGMENT = {}".format(result))

            if search[0] == search[1] and search[0] not in ["Black", "Undefined", "Brown", "White"]:
                last_same_color2 = search[0]

            if search[0] == search[1] == "Black":
                robot.stop_motors()
                counter = 0
                while counter < 10:
                    search = robot.sensor_data("ColorSensor")
                    if search[0] == search[1] == "Black":
                        counter += 1
                    else:
                        break
                if counter >= 10:
                    # print("ESSEEEE EH O CASO!")
                    # print("REVERSE_PATH = {}".format(robot.reverse_path))
                    # print(last_same_color2)
                    return_last_color(robot, last_same_color2, [])
                    robot.rotate(-180, speed=400)
                    #robot.move_timed(0.3,direction="back")
                    while True:
                        search = robot.sensor_data("ColorSensor")
                        if search[0] == last_same_color2:
                            robot.stop_motors()
                            break
                        else:
                            robot.motors.left.run_forever(speed_sp=-500)
                    while True:
                        search = robot.sensor_data("ColorSensor")
                        if search[1] == last_same_color2:
                            robot.stop_motors()
                            break
                        else:
                            robot.motors.right.run_forever(speed_sp=-500)
                    robot.move_timed(how_long=0.2, direction="back")
                    # print("REVERSE_PATH = {}".format(robot.reverse_path))
                    # print("ESSEEEE EH O CASO FIM!")

                else:
                    # print('ELE TENTOU ENTRAR NO CASO DO BLACK QUANDO NÃO DEVIA')
                    pass

            # coloquei no inicio do while
            # if robot.sensor_data("Ultrasonic") < 17 and not robot.has_doll:
            #     # robot.stop_motors()
            #     rescue(robot)
            #     # for i in range(2):
            #     #     print('IDENTIFICOU O BONECO')
            #     #     ev3.Sound.beep().wait()
            #     # time.sleep(2)

            white_counter = 0
            # print(robot.historic)


            if result == "On square" or robot.in_rect:
                # print("On square")

                if not im_learning:
                    if robot.rect_color not in ["White", "Undefined", "Black"]:
                        try:
                            # print("Tentando executar acao para a cor:", robot.rect_color)
                            # print("Aprendidos ate agora:", robot.learned_colors)

                            if robot.learned_colors[robot.rect_color] and\
                                    robot.sensor_data("ColorSensor") not in ["Black", "Brown", "Undefined"]:
                                # print("Executando acao:", robot.learned_colors[robot.rect_color])
                                robot.run_action(robot.learned_colors[robot.rect_color][0], im_learning)

                                # adds action to historic
                                if robot.reverse_path is None:
                                    robot.historic.append(robot.rect_color)
                                elif robot.reverse_path and robot.nao_pode:
                                    # print("Reverse path is True", len(robot.historic))
                                    # print("AAAAAAAAAA")
                                    if len(robot.historic) == 1:
                                        pass
                                    else:
                                        # print("Removendo item")
                                        last_item = robot.historic[-1]
                                        robot.historic.pop()
                                        # Uso de pilha para detectar começo de pista
                                        # if len(robot.historic) == 1:
                                        #     #print("Ultimo item")
                                        #     robot.rotate(90)
                                        #     robot.reverse_path = None
                                        #     robot.historic.append(last_item)

                                robot.move_timed(how_long=0.4)
                                color = 0

                                # time.sleep(5.5)

                        except:
                            being_learned = robot.rect_color

                            # abre arquivo de learning dic e verifica se o arquivo possui a key em questão
                            with open(robot.fixed_file_name) as json_file:
                                temp_learning_dic = json.load(json_file)
                                print("Abre arquivo de learning dic e verifica se o"
                                      " arquivo possui a key em questao, em arquivo:", temp_learning_dic)

                            if being_learned in temp_learning_dic.keys():
                                print("Possui key em questao salva no json!")
                                robot.has_came_from_json = True
                                learning_dic[being_learned] = temp_learning_dic[being_learned]
                                print("learning_dic:", learning_dic)
                            else:
                                # print("Ação para a cor:", robot.rect_color, "nao existe ou falhou!")
                                print("Nao possui key em questao no json!")
                                learning_dic[being_learned] = ["right", "forward", "left"]

                            im_learning = True

                elif im_learning:
                    robot.run_action(learning_dic[being_learned][0], im_learning)
                    while True:
                        if robot.sensor_data("Ultrasonic") < 15 and not robot.has_doll:
                            robot.stop_motors()
                            rescue(robot)
                            time.sleep(1)

                        robot.update()
                        search = robot.sensor_data("ColorSensor")
                        result = color_realignment(robot, search, robot.infrared_sensors)
                        # print("COLOR_REALIGMENT = {}".format(result))

                        if robot.sensor_data("ColorSensor")[0] == robot.sensor_data("ColorSensor")[1] and \
                                robot.sensor_data("ColorSensor")[1] == "White":
                            white_counter += 1

                        color_sensor = robot.sensor_data("ColorSensor")

                        if color_sensor[0] == color_sensor[1] and \
                                color_sensor[0] not in ["White", "Undefined", "Black", "Brown"]:
                            if color_sensor[0] != being_learned or (
                                    color_sensor[0] == being_learned and white_counter >= 5):

                                robot.learned_colors[being_learned] = [learning_dic[being_learned][0], 1]

                                # salva aprendizado finalizado em arquivo json
                                with open(robot.file_name, 'w') as outfile:
                                    json.dump([robot.learned_colors, True], outfile)

                                # adds action to historic
                                # if robot.reverse_path == None and not robot.nao_pode:
                                #     #print("BBBBBBBBB")
                                #     robot.historic.append(being_learned)

                                im_learning = False
                                being_learned = "Undefined"
                                learning_dic = {}

                                # print("Aprendi uma nova cor, segue o dicionario:", robot.learned_colors)

                                break

                        if robot.sensor_data("ColorSensor")[0] == "Black" and robot.sensor_data("ColorSensor")[1] == "Black":
                            # print("Wrong path")
                            robot.motors.left.stop()
                            robot.motors.right.stop()
                            time.sleep(0.1)
                            robot.realigment_counter = 0

                            # abre json e salva informações já armazenadas
                            with open(robot.fixed_file_name) as json_file:
                                json_learning_dic = json.load(json_file)

                            last_choise = learning_dic[being_learned][0]
                            del learning_dic[being_learned][0]

                            # salva arquivo json com as opções que não foram eliminadas
                            json_learning_dic[being_learned] = learning_dic[being_learned]
                            with open(robot.fixed_file_name, 'w') as outfile:
                                print("Salvando JSON novamente! Geratriz:", json_learning_dic)
                                json.dump(json_learning_dic, outfile)


                            return_last_color(robot, being_learned, last_choise)
                            # print("learned_dic =", learning_dic)
                            break

                    white_counter = 0

            # if search[0] == "Undefined" and search[1] == "Undefined":
            #     undefined_counter += 1
            #
            #     if undefined_counter > 5:
            #         undefined_counter = 0
            #         #print("Both sensors are undefined!")
            #         robot.move_timed(how_long=0.5, direction="back")
            #         robot.motors.left.stop()
            #         robot.motors.right.stop()

    except KeyboardInterrupt:
        robot.motors.right.stop()
        robot.motors.left.stop()
        robot.motors.alternative.stop()
        server.client.loop_stop()
        server.client.disconnect()

try:
    if __name__ == '__main__':
        # robot.reverse_path = True
        # robot.learned_colors = {'Red': ['left', 1], 'Green': ['forward', 0], 'Blue': ['right', 0]}
        # robot.primeiro_bounding_box = False
        # robot.has_doll = True
        # bounding_box(robot)
        main()
        """
        FOI PARA O TESTE DO ERRO DA FOTO 1,2
        while True:
            if robot.sensor_data("Ultrasonic") < 15 and not robot.has_doll:
                rescue(robot)
            else:
                print("CORRETO")
        """
except KeyboardInterrupt:
    robot.motors.right.stop()
    robot.motors.left.stop()
    robot.motors.alternative.stop()
    server.client.loop_stop()
    server.client.disconnect()
