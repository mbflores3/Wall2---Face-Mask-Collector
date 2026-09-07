
import serial
import time

from utils import SmallestSignedAngleBetween

#######################################################
##### CODIGO PARA QUE EL HAGA PICK DE MASCARILLAS #####
#######################################################

#PARÁMETROS DE COMUNICACIÓN
# ser_ard = serial.Serial("/dev/ttyACM0", baudrate=115200) #Modificar el puerto serie de ser necesario
# ser_esp = serial.Serial("/dev/ttyUSB0", baudrate=115200) #Modificar el puerto serie de ser necesario

ser_ard = serial.Serial("COM6", baudrate=115200) #Modificar el puerto serie de ser necesario


# ser_esp.reset_input_buffer()
ser_ard.reset_input_buffer()



##### MODO DE OPERACIÓN DE WALL2 #####
modo = "PICK"



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

    print("----------- INICIANDO MICROCONTROLADORES ------------")
    # while(ser_ard.in_waiting <= 0): pass
    time.sleep(1)
    print("----------- COMENZANDO LECTURAS ------------")
    time.sleep(0.5)

    while True:

        init_loop = time.time()

        time.sleep(0.1)

        if(modo == "PICK"):
            Tw += time.time() - init_loop
            print("ESTOY ESPERANDO, ME QUEDAN %f SEGUNDOS EN ESTE MODO"  %(Tw_limit - Tw))
            if (Tw >= Tw_limit):
                comando = "PICK"
                comando = comando + "\n"
                comandoBytes = comando.encode()
                ser_ard.write(comandoBytes)
                Tw = 0
                print("MOVIENDO BRAZO PARA RECOGER MASCARILLA")
                time.sleep(5)

        




       



except KeyboardInterrupt:
    print("\nInterrupcion por teclado")
except ValueError as ve:
    print(ve)
    print("Otra interrupcion")
finally:
    ser_ard.close()
    # ser_esp.close()
