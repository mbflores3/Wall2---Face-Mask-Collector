
import serial
import time

from utils import SmallestSignedAngleBetween

############################################
##### CODIGO PARA QUE EL ROBOT EXPLORE #####
############################################


debounce = 0
debounce_giro = 0

#PARÁMETROS DE COMUNICACIÓN
# ser_ard = serial.Serial("/dev/ttyACM0", baudrate=115200) #Modificar el puerto serie de ser necesario
# ser_esp = serial.Serial("/dev/ttyUSB0", baudrate=115200) #Modificar el puerto serie de ser necesario

ser_ard = serial.Serial("COM6", baudrate=115200) #Modificar el puerto serie de ser necesario
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

kp = 5.8
kd = 0.001
ki = 1.2

kp_1 = 4.2
kd_1 = 0.001
ki_1 = 0.5

err = 0
err1 = 0
err2 = 0

# TIEMPO DE DISCRETIZACIÓN
Tm = 0.01

# TIEMPO DE ESPERA INICIAL
Tw = 0.0
Tw_limit = 2.0

# TIEMPO DE RECORRER INICIAL
Tr = 0.0
Tr_limit = 3.0


try:
    print("----------- INICIANDO WALL2 MARK5 ------------")
    comando = "RESET"
    comando = comando + "\n"
    comandoBytes = comando.encode()
    # ser_esp.write(comandoBytes)
    ser_ard.write(comandoBytes)

    time.sleep(0.5)

    print("----------- INICIANDO MICROCONTROLADORES ------------")
    while(ser_ard.in_waiting <= 0): pass
    time.sleep(1)
    print("----------- COMENZANDO LECTURAS ------------")
    time.sleep(0.5)

    while True:

        init_loop = time.time()

        time.sleep(0.000001)

        if(ser_ard.in_waiting > 0):
            line = ser_ard.readline().decode('utf-8').rstrip().split(":")
            if("YAW" in line[0]):
                yaw = float(line[1])
    

        pos_wall2 = (pos_1 + pos_2 + pos_3 + pos_4) / 4

        if(modo == "ESPERA"):
            Tw += time.time() - init_loop
            print("ESTOY ESPERANDO, ME QUEDAN %f SEGUNDOS EN ESTE MODO"  %(Tw_limit - Tw))
            if (Tw >= Tw_limit):
                modo = "EXPLORACION"

        if(modo == "EXPLORACION"):

            Tr += time.time() - init_loop

            if(dist < 50):
                modo = "OBSTACULO"

            if(Tr >= Tr_limit - 0.2 * (ciclo//3)):
                accion_exploracion = "GIRAR"
                ciclo += 1
                ref = ang_ref[(ciclo) % 4]
                dist_recor = pos_wall2
                Tr = 0


            err = SmallestSignedAngleBetween(yaw, ref)

            if(accion_exploracion == "AVANZAR"):
                pwm_base = 100
            elif(accion_exploracion == "GIRAR"):
                debounce_giro += 1
                pwm_base = 0
                if(debounce_giro > 50):
                    accion_exploracion = "AVANZAR"
                    dist_recor = pos_wall2
                    debounce_giro = 0
                    pwm_base = 100
                    Tr = 0

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

            comando = "SETRIGHT" + str(pwmR)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)

            print("POS WALL2: ", pos_wall2, "Tiempo: ", Tr, "REF: ", ref,  "YAW: ", yaw, "pwmL: ", pwmL, "pwmR: ", pwmR)



        if(modo == "OBSTACULO"):

            print("NOS HEMOS ENCONTRADO CON UN OBSTACULO, DETENER ROBOT AHORA!")
            comando = "SETLEFT" + str(0)
            comando = comando + "\n"
            comandoBytes = comando.encode()
            ser_ard.write(comandoBytes)

            if(dist > 50):
                modo = "EXPLORACION"

       



except KeyboardInterrupt:
    print("\nInterrupcion por teclado")
except ValueError as ve:
    print(ve)
    print("Otra interrupcion")
finally:
    ser_ard.close()
    # ser_esp.close()
