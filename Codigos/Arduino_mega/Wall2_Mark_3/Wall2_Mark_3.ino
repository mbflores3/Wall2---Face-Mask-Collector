/*
	Capitulo 65 de Arduino desde cero en Español.
	Visualizacion por monitor serie de las lecturas del MPU6050 para yaw, pitch y roll.
	Intercala texto si el pitch es mayor a 10 grados o menor a -10 grados a modo de ejemplo
	para tomar una accion.
	Requiere librerias I2cdev y Simple_MPU6050

	Codigo basado en el programa de ejemplo incluido en la libreria Simple_MPU6050

	https://www.youtube.com/c/BitwiseAr
	Autor: bitwiseAr  

*/

#include "Simple_MPU6050.h"					// incluye libreria Simple_MPU6050
#include <Servo.h>                  // Libreria para los servo

#define PI 3.1415926535897932384626433832795

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

int pwmBL = 0;
int pwmBR = 0;
int pwmTL = 0;
int pwmTR = 0;

//--------------- START -------------------
int control = 0;

//------------------ PWM ----------------------
int pwm_base = 0;
float pwm = 0;

//------ PID -----
float ref = 0.0;

float kp = 16.2;
float kd = 0.1;
float ki = 2.5;

float kp_1 = 8.2;
float kd_1 = 0.1;
float ki_1 = 1.5;

float err = 0;
float err1 = 0;
float err2 = 0;


//---------- TIEMPO DE MUESTREO -----------
float Tm = 0.1;

//---------- RECEPCION DE DATOS ------------
int angulo = 0;           // Variable para guardar el angulo que deseamos de giro de servomotores
String leeCadena;         // Almacena la cadena de datos recibida
String motor;

//----------- VARIABLES SERVOMOTORES -----------
int pulsoMinimo = 580;    // Duración en microsegundos del pulso para girar 0º
int pulsoMaximo = 2500;   // Duración en microsegundos del pulso para girar 180º
int pulsoMaximo2 = 3460;   // Duración en microsegundos del pulso para girar 270º
int a_g = 110;           // Variable para guardar el angulo que deseamos de giro en la garra
int a_m = 160;           // Variable para guardar el angulo que deseamos de giro en la muñeca
int a_b = 70;           // Variable para guardar el angulo que deseamos de giro en la base
int a_ant_g = 110;           // Variable para guardar el angulo anterior que deseamos de giro en la garra
int a_ant_m = 160;           // Variable para guardar el angulo anterior que deseamos de giro en la muñeca
int a_ant_b = 70;           // Variable para guardar el angulo anterior que deseamos de giro en la base
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

//------------ GIROSCOPIO ------------
Simple_MPU6050 mpu;				// crea objeto con nombre mpu
// ENABLE_MPU_OVERFLOW_PROTECTION();		// activa proteccion, ya no se requiere

// #define OFFSETS  -5114,     484,    1030,      46,     -14,       6  // Colocar valores personalizados

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
    Serial.printfloatx(F("Ref")  , ref, 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje Z, yaw
    Serial.printfloatx(F("Yaw")  , xyz[0], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje Z, yaw
    // Serial.printfloatx(F("Pitch"), xyz[1], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje Y, pitch
    // Serial.printfloatx(F("Roll") , xyz[2], 9, 4, F(",   "));  // muestra en monitor serie rotacion de eje X, roll
    // Serial.println();				// salto de linea

    while (Serial.available()) 
    {    // Leer el valor enviado por el Puerto serial
      char c  = Serial.read();     // Lee los caracteres
      if(isAlpha(c))
      {
        motor += c;
      }
      else
      {
        leeCadena += c;              // Convierte Caracteres a cadena de caracteres
      }
    }  
    if (leeCadena.length()>1)
    {
      if(motor == "R")
      { 
        ref = leeCadena.toFloat();   // Convierte Cadena de caracteres a decimal 
      }
      if(motor == "BASE")
      {
        pwm_base = leeCadena.toInt();
      }
      else
      {
        angulo = leeCadena.toInt();   // Convierte Cadena de caracteres a Enteros
        Serial.println("");
        Serial.print(angulo);         // Envia valor en Grados 
        Serial.println(" Grados");
        Serial.print("Tipo de motor: ");
        Serial.println(motor);
        delay(5);
      }
    }

    // err = ref - xyz[0];
    err = SmallestSignedAngleBetween(xyz[0], ref);
    // Serial.println(err);

    if(control == 1 && pwm_base != 0)
    {
      pwm = pwm + (kp + kd/Tm)*err + (-kp + ki * Tm - 2*kd/Tm)*err1 + (kd/Tm)*err2;

      err2 = err1;
      err1 = err;
    }

    else if(control == 1 && pwm_base == 0)
    {
      pwm = pwm + (kp_1 + kd_1/Tm)*err + (-kp_1 + ki_1 * Tm - 2*kd_1/Tm)*err1 + (kd_1/Tm)*err2;

      err2 = err1;
      err1 = err;
    }

    pwmBL = pwm_base - int(pwm);
    pwmBR = pwm_base + int(pwm);
    pwmTL = pwm_base - int(pwm);
    pwmTR = pwm_base + int(pwm);

    if (pwmBL < -255)
    {
      pwmBL = -255;
    }

    if (pwmBR < -255)
    {
      pwmBR = -255;
    }

    if (pwmTL < -255)
    {
      pwmTL = -255;
    }

    if (pwmTR < -255)
    {
      pwmTR = -255;
    }

    if (pwmBL > 255)
    {
      pwmBL = 255;
    }

    if (pwmBR > 255)
    {
      pwmBR = 255;
    }

    if (pwmTL > 255)
    {
      pwmTL = 255;
    }

    if (pwmTR > 255)
    {
      pwmTR = 255;
    }

    Serial.printfloatx(F("Pwm") , pwm, 9, 4, F(",   "));
    Serial.print("PWM MOTOR BR: ");
    Serial.print(pwmBR);
    Serial.print(", PWM MOTOR TR: ");
    Serial.print(pwmTR);
    Serial.print(", PWM MOTOR BL: ");
    Serial.print(pwmBL);
    Serial.print(", PWM MOTOR TL: ");
    Serial.println(pwmTL);
    Serial.println();
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
  Serial.println(F("Inicio:"));		// muestra texto estatico
#ifdef OFFSETS								// si existen OFFSETS
  Serial.println(F("Usando Offsets predefinidos"));			// texto estatico
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);	// inicializacion de sensor

#else										// sin no existen OFFSETS
  Serial.println(F(" No se establecieron Offsets, haremos unos nuevos.\n"	// muestra texto estatico
                   " Colocar el sensor en un superficie plana y esperar unos segundos\n"
                   " Colocar los nuevos Offsets en #define OFFSETS\n"
                   " para saltar la calibracion inicial \n"
                   " \t\tPresionar cualquier tecla y ENTER"));
  while (Serial.available() && Serial.read());		// lectura de monitor serie
  while (!Serial.available());   			// si no hay espera              
  while (Serial.available() && Serial.read()); 		// lecyura de monitor serie
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();	// inicializacion de sensor
#endif
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

  Serial.println("-----Iniciando post calibración-----");
  Serial.println("Llevando al brazo a posición inicial");
  servo1.write(a_g);
  servo2.write(a_m);
  servo3.write(a_b);
  servo4.write(180 - a_b);
}

void loop()
{
  mpu.dmp_read_fifo();

  if(motor == "START")
  {
    control = 1;
    // pwm_base = 80;
  }

  if(control == 1)
  {
    setBLspeed(pwmBL);
    setTLspeed(pwmTL);
    setBRspeed(pwmBR);
    setTRspeed(pwmTR);
  }

  if(motor == "G")
  {
    setGripper(angulo);
  }

  if(motor == "M")
  {
    setMuneca(angulo);
  }

  if(motor == "B")
  {
    setBase(angulo);
  }

  if(motor == "PICK")
  {
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
    setBase(70);
    setGripper(120);
  }

  if(motor == "RESET")
  {
    void(* resetSoftware)(void) = 0;
  }

  if(motor == "PAUSE")
  {
    pwm_base = 0;
  }

  if(motor == "STOP")
  {
    control = 0;
  }


  leeCadena = "";
  motor = "";
  angulo = 0;
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

//------------- FUNCION PARA CALCULAR ERROR ---------------
float SmallestSignedAngleBetween(float th1, float th2)
{
  float err = th2 - th1;

  if(err > 180)
  {
    err = -(360 - abs(th2) - abs(th1));
  }

  if(err < -180)
  {
    err = -(360 - abs(th2) - abs(th1));
  }

  return err;


}






