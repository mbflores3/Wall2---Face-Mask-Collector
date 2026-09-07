import serial
import time
import cv2
import numpy as np
import imutils
from imutils.video import FPS, WebcamVideoStream

from utils import hsv_masking, hsv_detection


##### PARÁMETROS PARA LA SEGMENTACIÓN #####
# upperThresh = np.array([115, 255, 255])
# lowerThresh = np.array([95, 35, 120])

upperThresh = np.array([111, 255, 255])
lowerThresh = np.array([91, 62, 61])

upperThresh_1 = np.array([237, 78, 204])
lowerThresh_1 = np.array([0, 0, 26])

cannyLowerThresh = 50

#PARÁMETROS DE COMUNICACIÓN
# ser_ard = serial.Serial("/dev/ttyACM0", baudrate=115200) #Modificar el puerto serie de ser necesario
# ser_esp = serial.Serial("/dev/ttyUSB0", baudrate=115200) #Modificar el puerto serie de ser necesario

ser_ard = serial.Serial("COM6", baudrate=115200) #Modificar el puerto serie de ser necesario
# ser_esp = serial.Serial("COM8", baudrate=115200) #Modificar el puerto serie de ser necesario


# ser_esp.reset_input_buffer()
ser_ard.reset_input_buffer()

debounce = 0

##### MODO DE OPERACIÓN DE WALL2 #####
modo = "ESPERA"
ajuste_camara = False


# PARÁMETROS DE CONTROL DE SEGUIMIENTO
ref_mask = (158, 284)
pos_mask = (0, 0)

kp_mask = 1.5
kd_mask = 0.005
ki_mask = 0.15


err_ang_mask = 0
err1_ang_mask = 0
err2_ang_mask = 0


avanceX = 0

# TIEMPO DE DISCRETIZACIÓN
Tm = 0.01


try:
    print("----------- INICIANDO WALL2 MARK5 ------------")
    comando = "RESET"
    comando = comando + "\n"
    comandoBytes = comando.encode()
    # ser_esp.write(comandoBytes)
    ser_ard.write(comandoBytes)

    time.sleep(0.5)

    print("----------- INICIANDO CAMARA ------------")

    height = 300
    stream = WebcamVideoStream(1)
    stream.start()
    # tracker = Tracker()
    fps = FPS().start()
    time.sleep(2)

    print("----------- INICIANDO MICROCONTROLADORES ------------")
    while(ser_ard.in_waiting <= 0): pass
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

        if(len(track_position) > 0):
        # Calculate node and draw finger
            cv2.circle(res1, tuple(track_position[0]), 6, (255, 255, 255), -1)
            cv2.putText(
                res1,
                "(%d, %d)" % (*track_position[0],),
                (abs(track_position[0][0] - 25), abs(track_position[0][1] - 25)),
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
        cv2.imshow("Live Feed", frame)
        cv2.imshow("Segmentation", res1)

        # Keyboard OP
        k = cv2.waitKey(5) & 0xFF
        if k == 27 or k == ord("q"):  # Esc
            break

        fps.update()

        if(len(track_position) > 0 and modo != "RECOLECCION"):
            debounce += 1
            if(debounce > 50):
                pos_mask = track_position[0]
                debounce = 0
                modo = "RECOLECCION"
        
        if(len(track_position) <= 0):
            modo = "ESPERA"
        

        elif(modo == "RECOLECCION"):

            pos_mask = track_position[0]
            err_ang_mask = ref_mask[0] - pos_mask[0]


            #PID X
            avanceX += (kp_mask + kd_mask/Tm)*err_ang_mask  + (-kp_mask + ki_mask * Tm - 2*kd_mask/Tm)*err1_ang_mask  + (kd_mask/Tm)*err2_ang_mask 
            err2_ang_mask = err1_ang_mask  
            err1_ang_mask = err_ang_mask  

            pwmL =  int(avanceX)
            pwmR = -int(avanceX)

            pwmL = int(pwmL) if( -255 <= pwmL <= 255) else int(255 * (abs(pwmL) / pwmL))
            pwmR = int(pwmR) if( -255 <= pwmR <= 255) else int(255 * (abs(pwmR) / pwmR))

            print("ERROR_X: ", err_ang_mask, "pwmL: ", pwmL, "pwmR: ", pwmR)

            if(abs(err_ang_mask) < 10):
                pwmL = 0
                pwmR = 0

            comando = "SETLEFT" + str(pwmL)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)

            time.sleep(0.0001)

            comando = "SETRIGHT" + str(pwmR)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)
        
        if(modo == "ESPERA"):
            print("ESPERANDO ENCONTRAR UNA MASCARILLA")

            comando = "SETVEL" + str(0)

            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)

            time.sleep(0.0001)




except KeyboardInterrupt:
    print("\nInterrupcion por teclado")
except ValueError as ve:
    print(ve)
    print("Otra interrupcion")
finally:
    ser_ard.close()
    # ser_esp.close()