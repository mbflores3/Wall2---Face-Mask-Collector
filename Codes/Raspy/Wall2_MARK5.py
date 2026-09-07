
import serial
import time
import cv2
import numpy as np
import imutils
from imutils.video import FPS, WebcamVideoStream

from utils import hsv_masking, hsv_detection, show, Tracker, set_ref, SmallestSignedAngleBetween


##### PARÁMETROS PARA LA SEGMENTACIÓN #####
upperThresh = np.array([115, 255, 255])
lowerThresh = np.array([95, 35, 120])

cannyLowerThresh = 50

debounce = 0
debounce_giro = 0

#PARÁMETROS DE COMUNICACIÓN
# ser_ard = serial.Serial("/dev/ttyACM0", baudrate=115200) #Modificar el puerto serie de ser necesario
# ser_esp = serial.Serial("/dev/ttyUSB0", baudrate=115200) #Modificar el puerto serie de ser necesario

ser_ard = serial.Serial("COM12", baudrate=115200) #Modificar el puerto serie de ser necesario
# ser_esp = serial.Serial("COM8", baudrate=115200) #Modificar el puerto serie de ser necesario


# ser_esp.reset_input_buffer()
ser_ard.reset_input_buffer()


ang_ref = [0, 90, 180, -90]
ciclo = 0

# VARIABLES DE SENSORES
dist = 100000
pos_1 = 0
pos_2 = 0
pos_3 = 0
pos_4 = 0
pos_wall2 = 0
dist_recor = 0
pwm_base = 50
pwm = 0

##### MODO DE OPERACIÓN DE WALL2 #####
modo = "ESPERA"
accion_exploracion = "AVANZAR"

# PARÁMETROS DE CONTROL DE ANGULO
ref = ang_ref[0]
yaw = 0

kp = 4.8
kd = 0.001
ki = 1.2

kp_1 = 1.2
kd_1 = 0.01
ki_1 = 0.5

err = 0
err1 = 0
err2 = 0

# PARÁMETROS DE CONTROL DE SEGUIMIENTO
ref_mask = (320, 320)
pos_mask = (0, 0)

kp_mask = 0.8
kd_mask = 0.001
ki_mask = 0.05

kp_1_mask = 0.5
kd_1_mask = 0.002
ki_1_mask = 0.015

err_dist_mask = 0
err1_dist_mask = 0
err2_dist_mask = 0

err_ang_mask = 0
err1_ang_mask = 0
err2_ang_mask = 0

avanceX = 0
avanceY = 0

# TIEMPO DE DISCRETIZACIÓN
Tm = 0.01

# TIEMPO DE ESPERA INICIAL
Tw = 0.0
Tw_limit = 10.0

try:
    print("----------- INICIANDO WALL2 MARK5 ------------")
    comando = "RESET"
    comando = comando + "\n"
    comandoBytes = comando.encode()
    # ser_esp.write(comandoBytes)
    ser_ard.write(comandoBytes)

    time.sleep(0.5)

    print("----------- INICIANDO CAMARA ------------")

    height = 400
    stream = WebcamVideoStream(0)
    stream.start()
    # tracker = Tracker()
    fps = FPS().start()
    time.sleep(2)

    print("----------- INICIANDO MICROCONTROLADORES ------------")
    # while(ser_ard.in_waiting <= 0): pass
    time.sleep(2)
    print("----------- COMENZANDO LECTURAS ------------")
    time.sleep(1)

    while True:

        init_loop = time.time()

        frame = stream.read()
        frame = imutils.resize(frame, height=height)
        fps.stop()

        # Detect Object
        hsv_mask, res1 = hsv_masking(frame, lowerThresh, upperThresh)

        # Track Object
        track_position = hsv_detection(hsv_mask)

        # Calculate node and draw finger
        for centroid in track_position:
            # Draw and label each finger
            cv2.circle(res1, tuple(centroid), 6, (255, 255, 255), -1)
            cv2.putText(
                res1,
                "(%d, %d)" % (*centroid,),
                (abs(centroid[0] - 25), abs(centroid[1] - 25)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
        cv2.putText(
            res1,
            "(%d, %d)" % (*ref_mask,),
            (abs(ref_mask[0] - 25), abs(ref_mask[1] - 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
        )
            

         # Update the frames
        cv2.imshow("Segmentation", res1)

        # Keyboard OP
        k = cv2.waitKey(5) & 0xFF
        if k == 27 or k == ord("q"):  # Esc
            break

        fps.update()

        if(len(track_position) > 0 and modo != "RECOLECCION"):
            debounce += 1
            if(debounce > 100):
                pos_mask = track_position[0]
                debounce = 0
                modo = "RECOLECCION"

        if(len(track_position) <= 0 and modo != "ESPERA"):
            modo = "EXPLORACION"
            dist_recor = pos_wall2


        if(ser_ard.in_waiting > 0):
            line = ser_ard.readline().decode('utf-8').rstrip().split(":")
            if("YAW" in line[0]):
                yaw = float(line[1])
        # if(ser_esp.in_waiting > 0):
        #     line = ser_esp.readline().decode('utf-8').rstrip().split(",")
        #     for words in line:
        #         words = words.split(":")
        #         if("M1" in words[0]):
        #             pos_1 = float(words[1])

        #         elif("M2" in words[0]):
        #             pos_2 = float(words[1])

        #         elif("M3" in words[0]):
        #             pos_3 = float(words[1])

        #         elif("M4" in words[0]):
        #             pos_4 = float(words[1])

        #         elif("DIST" in words[0]):
        #             dist = float(words[1])

        pos_wall2 = (pos_1 + pos_2 + pos_3 + pos_4) / 4
        # print("POS M1: ", pos_1, "POS M2: ", pos_2, "POS M3: ", pos_3, "POS M4: ", pos_4, "DIST: ", dist, "YAW: ", yaw)

        if(modo == "ESPERA"):
            Tw += time.time() - init_loop
            print("ESTOY ESPERANDO, ME QUEDAN %f SEGUNDOS EN ESTE MODO"  %(Tw_limit - Tw))
            if (Tw >= Tw_limit):
                modo = "EXPLORACION"

        if(modo == "EXPLORACION"):

            if(pos_wall2 - dist_recor >= 4.0 - 0.3 * (ciclo//3)):
                accion_exploracion = "GIRAR"
                ciclo += 1
                ref = ang_ref[(ciclo) % 4]
                dist_recor = pos_wall2


            err = SmallestSignedAngleBetween(yaw, ref)

            if(accion_exploracion == "AVANZAR"):
                pwm_base = 70
            elif(accion_exploracion == "GIRAR"):
                debounce_giro += 1
                pwm_base = 0
                if(err < 10 and debounce_giro > 100):
                    accion_exploracion == "AVANZAR"
                    dist_recor = pos_wall2
                    debounce_giro = 0

            if(pwm_base != 0):
                pwm = pwm + (kp + kd/Tm)*err + (-kp + ki * Tm - 2*kd/Tm)*err1 + (kd/Tm)*err2

            elif(pwm_base == 0):
                pwm = pwm + (kp_1 + kd_1/Tm)*err + (-kp_1 + ki_1 * Tm - 2*kd_1/Tm)*err1 + (kd_1/Tm)*err2

            err2 = err1
            err1 = err

            pwmL = pwm_base - int(pwm)
            pwmR = pwm_base + int(pwm)

            pwmL = int(pwmL) if( -255 <= pwmL <= 255) else int(255 * (abs(pwmL) / pwmL))
            pwmR = int(pwmR) if( -255 <= pwmR <= 255) else int(255 * (abs(pwmR) / pwmR))

            comando = "SETLEFT" + str(pwmL)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)

            time.sleep(0.0001)

            comando = "SETRIGHT" + str(pwmL)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)

            print("POS WALL2: ", pos_wall2, "DIST: ", dist, "REF: ", ref,  "YAW: ", yaw, "pwmL: ", pwmL, "pwmR: ", pwmR)

        elif(modo == "RECOLECCION"):

            pos_mask = track_position[0]

            err_dist_mask = ref_mask[1] - pos_mask[1]
            err_ang_mask = ref_mask[0] - pos_mask[0]

            #PID X
            avanceX += (kp_mask + kd_mask/Tm)*err_ang_mask  + (-kp_mask + ki_mask * Tm - 2*kd_mask/Tm)*err1_ang_mask  + (kd_mask/Tm)*err2_ang_mask 
            err2_ang_mask = err1_ang_mask  
            err1_ang_mask = err_ang_mask  

            #PID Y
            avanceY += (kp_1_mask + kd_1_mask/Tm)*err_dist_mask + (-kp_1_mask + ki_1_mask * Tm - 2*kd_1_mask/Tm)*err1_dist_mask + (kd_1_mask/Tm)*err2_dist_mask
            err2_dist_mask = err1_dist_mask
            err1_dist_mask = err_dist_mask

            pwmL = avanceY - int(avanceX)
            pwmR = avanceY + int(avanceX)

            pwmL = int(pwmL) if( -255 <= pwmL <= 255) else int(255 * (abs(pwmL) / pwmL))
            pwmR = int(pwmR) if( -255 <= pwmR <= 255) else int(255 * (abs(pwmR) / pwmR))

            print("ERROR_X: ", err_ang_mask, "ERROR_Y: ", err1_dist_mask, "pwmL: ", pwmL, "pwmR: ", pwmR)

            comando = "SETLEFT" + str(pwmL)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)

            time.sleep(0.0001)

            comando = "SETRIGHT" + str(pwmL)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)




except KeyboardInterrupt:
    print("\nInterrupcion por teclado")
except ValueError as ve:
    print(ve)
    print("Otra interrupcion")
finally:
    ser_ard.close()
    # ser_esp.close()
