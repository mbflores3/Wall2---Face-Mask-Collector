
//------------------ DEFINICIÓN PINES MOTORES ----------------
// MOTOR 1
#define ENCODER_A0  21  // cable amarillo pololu
#define ENCODER_B0  22  // cable blanco pololu  //cable azul va a 5-12V, cable verde y negro a tierra, cable rojo 12V 

// MOTOR 2
#define ENCODER_A1 35  // cable amarillo pololu
#define ENCODER_B1 34   // cable blanco pololu  //cable azul va a 5-12V, cable verde y negro a tierra, cable rojo 12V 

// MOTOR 3
#define ENCODER_A2 15  // cable amarillo pololu
#define ENCODER_B2 2   // cable blanco pololu  //cable azul va a 5-12V, cable verde y negro a tierra, cable rojo 12V 

// MOTOR 4
#define ENCODER_A3 19  // cable amarillo pololu
#define ENCODER_B3 18   // cable blanco pololu  //cable azul va a 5-12V, cable verde y negro a tierra, cable rojo 12V 

//------------------ DEFINICIÓN PINES ULTRASONIDO ----------------
#define ECHO 14
#define TRIG 27

//----------------- VARIABLES DE INTERRUPCIÓN ------------------------
//Variable global de posición compartida con la interrupción motor 1
volatile long theta0 = 0;
long newposition0 = 0;
float position0 = 0;

//Variable global de posición compartida con la interrupción motor 2
volatile long theta1 = 0;
long newposition1 = 0;
float position1 = 0;

//Variable global de posición compartida con la interrupción motor 3
volatile long theta2 = 0;
long newposition2 = 0;
float position2 = 0;

//Variable global de posición compartida con la interrupción motor 4
volatile long theta3 = 0;
long newposition3 = 0;
float position3 = 0;

//Variable global de pulsos compartida con la interrupción
volatile long pulsos0 = 0;
volatile long pulsos1 = 0;
volatile long pulsos2 = 0;
volatile long pulsos3 = 0;
long newpulsos0 = 0;
long newpulsos1 = 0;
long newpulsos2 = 0;
long newpulsos3 = 0;

//Variables de sensor ultrasonico
long duracion, distancia;

//Variables de debouncing
volatile unsigned long tiempoDeInterrupcionAnteriorA0 = 0;
volatile unsigned long tiempoDeInterrupcionAnteriorB0 = 0;
volatile unsigned long tiempoDeInterrupcionAnteriorA1 = 0;
volatile unsigned long tiempoDeInterrupcionAnteriorB1 = 0;
volatile unsigned long tiempoDeInterrupcionAnteriorA2 = 0;
volatile unsigned long tiempoDeInterrupcionAnteriorB2 = 0;
volatile unsigned long tiempoDeInterrupcionAnteriorA3 = 0;
volatile unsigned long tiempoDeInterrupcionAnteriorB3 = 0;
#define tiempoDeRebote 700 

//--------------------------------------------------------------------

//----------------------- VARIABLES GLOBALES GENERALES -------------------
//Tiempo transcurrido y resolcuion de los motores
unsigned long timeold;
float resolution = 1920.0;


//Variable Global Posicion y radio
float radio = 0.064;
float pi = 3.14159265359;

//------- RECEPCION DE DATOS DEDE ARDUINO---------
float pwm_rec = 0;
String leeCadena;         // Almacena la cadena de datos recibida
//----------------------------------------------------------------------

void setup()
{
  Serial.begin (115200);
  Serial.println("start");               
  timeold = 0;

  attachInterrupt(ENCODER_A0, leerEncoder_A0, CHANGE);
  attachInterrupt(ENCODER_B0, leerEncoder_B0, CHANGE);
  attachInterrupt(ENCODER_A1, leerEncoder_A1, CHANGE);
  attachInterrupt(ENCODER_B1, leerEncoder_B1, CHANGE);
  attachInterrupt(ENCODER_A2, leerEncoder_A2, CHANGE);
  attachInterrupt(ENCODER_B2, leerEncoder_B2, CHANGE);
  attachInterrupt(ENCODER_A3, leerEncoder_A3, CHANGE);
  attachInterrupt(ENCODER_B3, leerEncoder_B3, CHANGE);

  // CONFIGURACIÓN PINES SENSOR ULTRASONICO
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  
}
void loop()
{
  float rpm0;
  float rpm1;
  float rpm2;
  float rpm3;

  float pos0;
  float pos1;
  float pos2;
  float pos3;

  newpulsos0 = pulsos0;
  newpulsos1 = pulsos1;
  newpulsos2 = pulsos2;
  newpulsos3 = pulsos3;

  // Serial.println("-----------------------");
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duracion = pulseIn(ECHO, HIGH);
  distancia = (duracion / 2) / 29;
  
  if (millis() - timeold >= 100)
  {
    timeold = millis();
    pos0 = (float(theta0 / resolution)) * radio * 2 * pi;
    rpm0 = float(10 * (60.0/ resolution ) * newpulsos0);
    pulsos0 = 0;
    pos1 = (float(theta1 / resolution)) * radio * 2 * pi;
    rpm1 = float(10 * (60.0/ resolution ) * newpulsos1);
    pulsos1 = 0;
    pos2 = (float(theta2 / resolution)) * radio * 2 * pi;
    rpm2 = float(10 * (60.0/ resolution ) * newpulsos2);
    pulsos2 = 0;
    pos3 = (float(theta3 / resolution)) * radio * 2 * pi;
    rpm3 = float(10 * (60.0/ resolution ) * newpulsos3);
    pulsos3 = 0;

    Serial.print("M1: ");
    Serial.print(pos0);
    Serial.print(", M2: ");
    Serial.print(pos1);
    Serial.print(", M3: ");
    Serial.print(pos2);
    Serial.print(", M4: ");
    Serial.print(pos3);
    Serial.print(", DIST: ");
    Serial.println(distancia);

  }

  leeCadena = "";

  
}

// -------------------------------------
// ----- FUNCIONES DE INTERRUPCION -----
// -------------------------------------
void leerEncoder_A0(){
  if(micros() - tiempoDeInterrupcionAnteriorA0 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos0++; //Incrementa una revolución
    
  //Lectura de Posición  
    int a = digitalRead(ENCODER_A0);
    int b = digitalRead(ENCODER_B0);

    if (a == b){
      theta0--;
    }
    else{
      theta0++;
    }
    tiempoDeInterrupcionAnteriorA0 = micros();
  }
}

void leerEncoder_B0(){
  if(micros() - tiempoDeInterrupcionAnteriorB0 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos0++; //Incrementa una revolución
      
    //Lectura de Posición  
    int a = digitalRead(ENCODER_A0);
    int b = digitalRead(ENCODER_B0);

    if (a == b){
      theta0++;
    }
    else{
      theta0--;
    }
    tiempoDeInterrupcionAnteriorB0 = micros();
  }
}

void leerEncoder_A1(){
  if(micros() - tiempoDeInterrupcionAnteriorA1 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos1++; //Incrementa una revolución
    
    //Lectura de Posición  
    int a = digitalRead(ENCODER_A1);
    int b = digitalRead(ENCODER_B1);

    if (a == b){
      theta1++;
    }
    else{
      theta1--;
    }
  tiempoDeInterrupcionAnteriorA1 = micros();
  }
}

void leerEncoder_B1(){
  if(micros() - tiempoDeInterrupcionAnteriorB1 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos1++; //Incrementa una revolución
    
    //Lectura de Posición  
    int a = digitalRead(ENCODER_A1);
    int b = digitalRead(ENCODER_B1);

    if (a == b){
      theta1--;
    }
    else{
      theta1++;
    }

    tiempoDeInterrupcionAnteriorB1 = micros();
  }
}

void leerEncoder_A2(){
  if(micros() - tiempoDeInterrupcionAnteriorA2 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos2++; //Incrementa una revolución
    
    //Lectura de Posición  
    int a = digitalRead(ENCODER_A2);
    int b = digitalRead(ENCODER_B2);

    if (a == b){
      theta2++;
    }
    else{
      theta2--;
    }
    tiempoDeInterrupcionAnteriorA2 = micros();
  }
}

void leerEncoder_B2(){
  if(micros() - tiempoDeInterrupcionAnteriorB2 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos2++; //Incrementa una revolución
    
    //Lectura de Posición  
    int a = digitalRead(ENCODER_A2);
    int b = digitalRead(ENCODER_B2);

    if (a == b){
      theta2--;
    }
    else{
      theta2++;
    }
    tiempoDeInterrupcionAnteriorB2 = micros();
  }
}

void leerEncoder_A3(){
  if(micros() - tiempoDeInterrupcionAnteriorA3 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos3++; //Incrementa una revolución
    
    //Lectura de Posición  
    int a = digitalRead(ENCODER_A3);
    int b = digitalRead(ENCODER_B3);

    if (a == b){
      theta3--;
    }
    else{
      theta3++;
    }
    tiempoDeInterrupcionAnteriorA3 = micros();
  }
}

void leerEncoder_B3(){
  if(micros() - tiempoDeInterrupcionAnteriorB3 > tiempoDeRebote)
  {
    //Lectura de Velocidad
    pulsos3++; //Incrementa una revolución
    
    //Lectura de Posición  
    int a = digitalRead(ENCODER_A3);
    int b = digitalRead(ENCODER_B3);

    if (a == b){
      theta3++;
    }
    else{
      theta3--;
    }
    tiempoDeInterrupcionAnteriorB3 = micros();
  }
}

