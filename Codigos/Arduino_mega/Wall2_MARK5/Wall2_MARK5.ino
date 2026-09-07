/*
  REFERENCIAS:

    -Codigo basado en el programa de ejemplo incluido en la libreria Simple_MPU6050
    https://www.youtube.com/c/BitwiseAr
    Autor: bitwiseAr  

*/

#include "Simple_MPU6050.h"					// incluye libreria Simple_MPU6050
#include <Servo.h>                  // Libreria para los servo

//----------------- PARÁMETROS MPU ------------------
#define MPU6050_ADDRESS_AD0_LOW     0x68			// direccion I2C con AD0 en LOW o sin conexion
#define MPU6050_ADDRESS_AD0_HIGH    0x69			// direccion I2C con AD0 en HIGH
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW	// por defecto AD0 en LOW

//----------------- PARÁMETROS SERVOS -----------------------
#define SERVO_1 3
#define SERVO_2 4
#define SERVO_3 5
#define SERVO_4 6

//------------------ MOTORES DC ---------------------
int ENB_TL = 8;
int IN3_TL = 24;
int IN4_TL = 22;

int IN1_TR = 25;
int IN2_TR = 23;
int ENA_TR = 9;

int ENB_BR = 10;
int IN3_BR = 30;
int IN4_BR = 32;

int IN1_BL = 31;
int IN2_BL = 33;
int ENA_BL = 11;

//---------- RECEPCION DE DATOS ------------
int number = 0;                   // Variable para guardar el int que se recibe
String inputString = "";          // Almacena la cadena de datos recibida
String numString = "";            // Almacena la cadena de datos recibida para números
bool stringComplete = false;

//----------- VARIABLES SERVOMOTORES -----------
int pulsoMinimo = 580;    // Duración en microsegundos del pulso para girar 0º
int pulsoMaximo = 2500;   // Duración en microsegundos del pulso para girar 180º
int pulsoMaximo2 = 3460;   // Duración en microsegundos del pulso para girar 270º
int a_g = 110;           // Variable para guardar el angulo que deseamos de giro en la garra
int a_m = 180;           // Variable para guardar el angulo que deseamos de giro en la muñeca
int a_b = 100;           // Variable para guardar el angulo que deseamos de giro en la base
int a_ant_g = 110;           // Variable para guardar el angulo anterior que deseamos de giro en la garra
int a_ant_m = 180;           // Variable para guardar el angulo anterior que deseamos de giro en la muñeca
int a_ant_b = 100;           // Variable para guardar el angulo anterior que deseamos de giro en la base
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

//------------ GIROSCOPIO ------------
Simple_MPU6050 mpu;				// crea objeto con nombre mpu
// ENABLE_MPU_OVERFLOW_PROTECTION();		// activa proteccion, ya no se requiere

#define OFFSETS  2610,   -2678,     850,     159,      36,      41  // Colocar valores personalizados

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
// spamtimer funcion para generar demora al escribir en monitor serie sin usar delay()

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);
// printfloatx funcion para mostrar en monitor serie datos para evitar el uso se multiples print()

// mostrar_valores funcion que es llamada cada vez que hay datos disponibles desde el sensor
void mostrar_valores (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {	
  uint8_t SpamDelay = 100;			// demora para escribir en monitor serie de 100 mseg
  Quaternion q;					// variable necesaria para calculos posteriores
  VectorFloat gravity;				// variable necesaria para calculos posteriores
  float ypr[3] = { 0, 0, 0 };			// array para almacenar valores de yaw, pitch, roll
  float xyz[3] = { 0, 0, 0 };			// array para almacenar valores convertidos a grados de yaw, pitch, roll
  spamtimer(SpamDelay) {			// si han transcurrido al menos 100 mseg entonces proceder
    mpu.GetQuaternion(&q, quat);		// funcion para obtener valor para calculo posterior
    mpu.GetGravity(&gravity, &q);		// funcion para obtener valor para calculo posterior
    mpu.GetYawPitchRoll(ypr, &q, &gravity);	// funcion obtiene valores de yaw, ptich, roll
    mpu.ConvertToDegrees(ypr, xyz);		// funcion convierte a grados sexagesimales
    Serial.print("YAW: ");
    Serial.println(xyz[0]);
  }
}

void setup() {
  uint8_t val;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE	// activacion de bus I2C a 400 Khz
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  
  Serial.begin(115200);			// inicializacion de monitor serie a 115200 bps
  while (!Serial); 			// espera a enumeracion en caso de modelos con USB nativo
  Serial.println(F("Iniciando sensor MPU6050"));		// muestra texto estatico
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);	// inicializacion de sensor

  mpu.on_FIFO(mostrar_valores);		// llamado a funcion mostrar_valores si memoria FIFO tiene valores

  pinMode (ENB_TL, OUTPUT);
  pinMode (IN3_TL, OUTPUT);
  pinMode (IN4_TL, OUTPUT);

  pinMode (IN1_TR, OUTPUT);
  pinMode (IN2_TR, OUTPUT);
  pinMode (ENA_TR, OUTPUT);

  pinMode (ENB_BR, OUTPUT);
  pinMode (IN3_BR, OUTPUT);
  pinMode (IN4_BR, OUTPUT);

  pinMode (IN1_BL, OUTPUT);
  pinMode (IN2_BL, OUTPUT);
  pinMode (ENA_BL, OUTPUT);

  servo1.attach(SERVO_1, pulsoMinimo, pulsoMaximo);
  servo2.attach(SERVO_2, pulsoMinimo, pulsoMaximo);
  servo3.attach(SERVO_3, pulsoMinimo, pulsoMaximo2);
  servo4.attach(SERVO_4, pulsoMinimo, pulsoMaximo2);
  Serial.begin(115200);

  Serial.println("Llevando al brazo a posición inicial");
  servo1.write(a_g);
  servo2.write(a_m);
  servo3.write(a_b);
  servo4.write(180 - a_b);
  inputString.reserve(200);
}

void loop()
{
  mpu.dmp_read_fifo();

  if (stringComplete) {//El comando fue recibido, procedemos a compararlo
    
    if (inputString.equals("PICK")){
      setMuneca(140);
      setBase(160);
      setGripper(5);
      setBase(80);
      setMuneca(45);
      setGripper(120);
      setMuneca(75);
      setMuneca(45);
      setMuneca(180);
      setBase(80);
      setBase(100);
      setGripper(120);
    }
    else if (inputString.equals("AJUSTARCAMARA")){
      setMuneca(180);
      setBase(125);
    }
    else if (inputString.equals("RESET")){
      void(* resetSoftware)(void) = 0;
    }
    else if (inputString.equals("SETLEFT")){
      setBLspeed(number);
      setTLspeed(number);
    }
    else if (inputString.equals("SETRIGHT")){
      setBRspeed(number);
      setTRspeed(number);
    }
    else if (inputString.equals("SETVEL")){
      setBLspeed(number);
      setTLspeed(number);
      setBRspeed(number);
      setTRspeed(number);
    }

    number = 0;                   
    inputString = ""; 
    numString = ""; 
    stringComplete = false;
  }
}		

//------------- FUNCION DE RECEPCION DE INFORMACION ------------
void serialEvent() {
  
  while (Serial.available()) {//Mientras tengamos caracteres disponibles en el buffer
    char inChar = (char)Serial.read();//Leemos el siguiente caracter
    if (inChar == '\n') {//Si el caracter recibido corresponde a un salto de línea
      stringComplete = true;//Levantamos la bandera 
    }
    else{//Si el caracter recibido no corresponde a un salto de línea
      if (isAlpha(inChar))
      {
        inputString += inChar;
      }
      else
      {
        numString += inChar;
      }
    }
  }
  if (numString.length()>1 && stringComplete)
  {
    number = numString.toInt();   // Convierte Cadena de caracteres a Enteros
    delay(5);
  }
}


//------------- FUNCIONES MOTORES TOP ---------------
void setTLspeed(int vel)
{
  if (vel < 0){
    digitalWrite (IN3_TL, HIGH);
    digitalWrite (IN4_TL, LOW);
  }
  else {
    digitalWrite (IN3_TL, LOW);
    digitalWrite (IN4_TL, HIGH);
    }
  analogWrite (ENB_TL, abs(vel));
}

void setTRspeed(int vel)
{  
  if (vel < 0){
    digitalWrite (IN1_TR, HIGH);
    digitalWrite (IN2_TR, LOW);
  }
  else {
    digitalWrite (IN1_TR, LOW);
    digitalWrite (IN2_TR, HIGH);
    }
  analogWrite (ENA_TR, abs(vel));
}

//------------- FUNCIONES MOTORES BOTTOM ---------------

void setBLspeed(int vel)
{
  if (vel < 0)
  {
    digitalWrite (IN1_BL, HIGH);
    digitalWrite (IN2_BL, LOW);
  }
  else 
  {
    digitalWrite (IN1_BL, LOW);
    digitalWrite (IN2_BL, HIGH);
  }
  analogWrite (ENA_BL, abs(vel));
}

void setBRspeed(int vel)
{
  if (vel < 0)
  {
    digitalWrite (IN3_BR, LOW);
    digitalWrite (IN4_BR, HIGH);
  }
  else
  {
    digitalWrite (IN3_BR, HIGH);
    digitalWrite (IN4_BR, LOW);
  }
  analogWrite (ENB_BR, abs(vel));
}
//------------- FUNCIONES SERVO PICK ---------------
void setGripper(int ang)
{
  a_g = ang;
  if(a_g > a_ant_g)
  {
    for(int angulo = a_ant_g; angulo <= a_g; angulo++)
    {
      servo1.write(angulo);
      delay(15);
    }
  }
  else if(a_g < a_ant_g)
  {
    for(int angulo = a_ant_g; angulo >= a_g; angulo--)
    {
      servo1.write(angulo);
      delay(15);
    }
  }
  a_ant_g = a_g;
}

void setMuneca(int ang)
{
  a_m = ang;
    if(a_m > a_ant_m)
    {
      for(int angulo = a_ant_m; angulo <= a_m; angulo++)
      {
        servo2.write(angulo);
        delay(15);
      }
    }
    else if(a_m < a_ant_m)
    {
      for(int angulo = a_ant_m; angulo >= a_m; angulo--)
      {
        servo2.write(angulo);
        delay(15);
      }
    }
    a_ant_m = a_m;
}

void setBase(int ang)
{
   a_b = ang;
    if(a_b > a_ant_b)
    {
      for(int angulo = a_ant_b; angulo <= a_b; angulo++)
      {
        servo3.write(angulo);
        servo4.write(180 - angulo);
        delay(15);
      }
    }
    else if(a_b < a_ant_b)
    {
      for(int angulo = a_ant_b; angulo >= a_b; angulo--)
      {
        servo3.write(angulo);
        servo4.write(180 - angulo);
        delay(15);
      }
    }
    a_ant_b = a_b;
}


