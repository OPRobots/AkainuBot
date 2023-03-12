#include <PIDfromBT.h>

//Pines de los motores
int MotorIalante = 4;
int MotorIatras = 5;
int MotorDalante = 8;
int MotorDatras = 7;
int PWM_MotorD = 9;
int PWM_MotorI = 3;

int estanbay = 6;

//Pines de los Sensores lellendolos de izquierda a derecha viendo el robot de la bateria a los sensores
int Sensor1 = A7;
int Sensor2 = A6;
int Sensor3 = A5;
int Sensor4 = A4;
int Sensor5 = A3;
int Sensor6 = A2;
int Sensor7 = A1;

//Pines de los interruptores para diferentes modos de velocidad
int interruptor1 = 10;
int interruptor2 = 11;
int interruptor3 = 12;

//Arrays de valor de sensores en negro en blanco y el umbral (utilizados en la calibracion)
int negros[7];
int blancos[7];
int umbrales[7];
// este array es un array intermediario para hacer operaciones no tiene relevancia en el programa
int lectura[7];

//Variables PID    SIN MILLIS
float kp = 0.8;   //1.65
float kd = 30;    //90
float ki = 0;     //-0.24
int posicion = 0;
int posicion_anterior = 0;
int velocidad = 0;
int integral = 0;
int correccion = 0;

//Variables barias
bool calibracion = true;
int i = 0;
int recivido = 0;
long millisAnterior = 0;

int velD = 0;
int velI = 0;
int vel = 125;    //150

PIDfromBT pid_calibrate(&kp, &ki, &kd, &vel, DEBUG);

void setup() {
  pinMode(MotorIalante, OUTPUT);
  pinMode(MotorIatras, OUTPUT);
  pinMode(MotorDalante, OUTPUT);
  pinMode(MotorDatras, OUTPUT);


  pinMode(Sensor1, INPUT);
  pinMode(Sensor2, INPUT);
  pinMode(Sensor3, INPUT);
  pinMode(Sensor4, INPUT);
  pinMode(Sensor5, INPUT);
  pinMode(Sensor6, INPUT);
  pinMode(Sensor7, INPUT);


  pinMode(interruptor1, INPUT_PULLUP);
  pinMode(interruptor2, INPUT_PULLUP);
  pinMode(interruptor3, INPUT_PULLUP);

  pinMode(13, OUTPUT);


  digitalWrite(MotorIalante, LOW);
  digitalWrite(MotorIatras, LOW);
  digitalWrite(MotorDalante, LOW);
  digitalWrite(MotorDatras, LOW);
  digitalWrite(estanbay, HIGH);

  Serial.begin(9600);
  delay(500);
}

void loop() {


  ///////////////////////////////////////////////////
  /*Comunicacion del bluethooth para calibracion del PID*/
  ///////////////////////////////////////////////////

  pid_calibrate.update();
  
/*
  if (Serial.available() >= 0) {
    recivido = Serial.read();
  }
  else {
    recivido = 0;
  }

  switch (recivido) {
    case 97:
      kp += 0.05;
      Serial.print("Kp: ");
      Serial.println(kp);
      break;
    case 115:
      kd += 1;
      Serial.print("Kd: ");
      Serial.println(kd);
      break;
    case 100:
      ki += 0.02;
      Serial.print("Ki: ");
      Serial.println(ki);
      break;
    case 122:
      kp -= 0.05;
      Serial.print("Kp: ");
      Serial.println(kp);
      break;
    case 120:
      kd -= 1;
      Serial.print("Kd: ");
      Serial.println(kd);
      break;
    case 99:
      ki -= 0.02;
      Serial.print("Ki: ");
      Serial.println(ki);
      break;
    case 105:
      vel += 1;
      Serial.print("Vel: ");
      Serial.println(vel);
      break;
    case 107:
      vel -= 1;
      Serial.print("Vel: ");
      Serial.println(vel);
      break;
  }

*/
  ///////////////////////////////////////////////////
  /*realizamos la calibracion inicial obteniendo los minimos y maximos leidos por cada sensor durante 5 seg y calculamos los umbrales*/
  ///////////////////////////////////////////////////

  if (calibracion == true) {
    digitalWrite(13, HIGH);
    millisAnterior = millis();
    do {

      // lectura de los sensores
      lectura[0] = analogRead(Sensor1);
      lectura[1] = analogRead(Sensor2);
      lectura[2] = analogRead(Sensor3);
      lectura[3] = analogRead(Sensor4);
      lectura[4] = analogRead(Sensor5);
      lectura[5] = analogRead(Sensor6);
      lectura[6] = analogRead(Sensor7);

      // registramos los valores Minimos y maximos en negros y blancos
      for (i = 0; i < 7; i++) {
        if (lectura[i] < negros[i]) {
          negros[i] = lectura[i];
        }
        if (lectura[i] > blancos[i]) {
          blancos[i] = lectura[i];
        }
      }

    } while ((millis() - millisAnterior) <= 5000);

    // Calculamos los umbrales para cada sensor
    for (i = 0; i < 7; i++) {
      umbrales[i] = ((negros[i] + blancos[i]) / 2) - 100;
    }

    for (i = 0; i < 7; i++) {
      Serial.print(umbrales[i]);
      Serial.print("   ");
    }
    Serial.println("   ");
    calibracion = false;
    digitalWrite(13, LOW);
  }





  ///////////////////////////////////////////////////
  /*Calculo del analogico de desplazamiento del array de sensores*/
  ///////////////////////////////////////////////////





  //cada 20 milisegundos lee los sensores y llama a la funcion que ejecuta el calculo para hacerlo de manera periodica

  if (millis() % 1 == 0) {
  lectura[0] = analogRead(Sensor1);
  lectura[1] = analogRead(Sensor2);
  lectura[2] = analogRead(Sensor3);
  lectura[3] = analogRead(Sensor4);
  lectura[4] = analogRead(Sensor5);
  lectura[5] = analogRead(Sensor6);
  lectura[6] = analogRead(Sensor7);

  //una secuencia de ifs para determinar cada posible situacion del robot y crear un psudoanalogico escalonado para meter al PID

  if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] <= umbrales[2] && lectura[3] <= umbrales[3] && lectura[4] <= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = 0; //

  } else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] <= umbrales[2] && lectura[3] <= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = -15; //

  } else if (lectura[0] >= umbrales[0] && lectura[1] <= umbrales[1] && lectura[2] <= umbrales[2] && lectura[3] <= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = -65; //

  } else if (lectura[0] >= umbrales[0] && lectura[1] <= umbrales[1] && lectura[2] <= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) { // salio del to por uno de los laos

    posicion = -80; //

  } else if (lectura[0] <= umbrales[0] && lectura[1] <= umbrales[1] && lectura[2] <= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = -95; //

  } else if (lectura[0] <= umbrales[0] && lectura[1] <= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = -145; //

  } else if (lectura[0] <= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6] && posicion <= 0 ) {

    posicion = -200; //

  }


  // ahora lo mismo pero por el otro lado del array


  else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] <= umbrales[3] && lectura[4] <= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = 15;

  } else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] <= umbrales[3] && lectura[4] <= umbrales[4] && lectura[5] <= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = 65;

  } else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] <= umbrales[4] && lectura[5] <= umbrales[5] && lectura[6] >= umbrales[6]) {

    posicion = 80;

  } else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] <= umbrales[4] && lectura[5] <= umbrales[5] && lectura[6] <= umbrales[6]) {

    posicion = 95;

  } else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] <= umbrales[5] && lectura[6] <= umbrales[6]) {

    posicion = 145;

  } else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] <= umbrales[6] && posicion >= 0) {

    posicion = 200;

  }

  //se salio por algun lado

  else if (lectura[0] >= umbrales[0] && lectura[1] >= umbrales[1] && lectura[2] >= umbrales[2] && lectura[3] >= umbrales[3] && lectura[4] >= umbrales[4] && lectura[5] >= umbrales[5] && lectura[6] >= umbrales[6]) {
    //
    if (posicion >= 0) {
      posicion = 250;
    } else {
      posicion = -250;
    }
  }


  ///////////////////////////////////////////////////
  //Calculo del PID
  ///////////////////////////////////////////////////


  velocidad = posicion - posicion_anterior;
  integral = integral + (posicion/10);
  //integral= constrain(integral,-500,500);
  
  correccion = ((kp * posicion) + (kd * velocidad) + (ki * integral));
  
/*  Serial.print(correccion);
  Serial.print(" = ");
  Serial.print(kp * posicion);
  Serial.print(" + ");
  Serial.print(kd * velocidad);
  Serial.print(" + ");
  Serial.println(ki * integral);
 */ 
  correccion = map(correccion, -800, 800, -255, 255);
  posicion_anterior = posicion;
  
  }

  
  /* Serial.print(correccion);
  if(velocidad>0){
   Serial.print("   ");
   Serial.println(velocidad);
  }
   Serial.println("   ");
   Serial.println(posicion);
   Serial.print("   ");
   Serial.println(correccion);
    }*/


  ///////////////////////////////////////////////////
  /*Asignacion de velocidades a los motores*/
  ///////////////////////////////////////////////////



  //aqui aplicamos la correccion del pid a las velocidades de los motores
  //de tal modo que si la correccion hace que una rueda se ponga a 255 a la otra se le aplique el doble de correccion
  
  /*if(correccion>(255-vel)){
  velD = vel - (correccion*2);
  velI = 255;
  Serial.println("TopeI");
  }else if(correccion <((255-vel)*(-1))){
  velD = 255;
  velI = vel + (correccion*2);
  Serial.println("TopeD");
  }else{
  velD = vel - correccion;
  velI = vel + correccion;
  }
*/


  velD = vel - correccion;
  velI = vel + correccion;
  
  //limitamos desbordamientos
  velD = constrain(velD, -255, 255);
  velI = constrain(velI, -255, 255);

  //asiganmos valores a la rueda derecha teniendo en cuenta de que si el valor es negativo va hacia atras
  if (velD >= 0) {
    digitalWrite(MotorDalante, HIGH);
    digitalWrite(MotorDatras, LOW);
    analogWrite(PWM_MotorD, velD);
  } else {
    digitalWrite(MotorDalante, LOW);
    digitalWrite(MotorDatras, HIGH);
    analogWrite(PWM_MotorD, abs(velD));
  }

  //asiganmos valores a la rueda izquierda teniendo en cuenta de que si el valor es negativo va hacia atras
  if (velI >= 0) {
    digitalWrite(MotorIalante, HIGH);
    digitalWrite(MotorIatras, LOW);
    analogWrite(PWM_MotorI, velI);
  } else {
    digitalWrite(MotorIalante, LOW);
    digitalWrite(MotorIatras, HIGH);
    analogWrite(PWM_MotorI, abs(velI));
  }
  /*Serial.print(velD);
  Serial.print("   ");
  Serial.println(velI);*/

  ///////////////////////////////////////
  //COMPROBACIÓN DE SENSORES
  ////////////////////////////////////////

  /*
  Serial.print(analogRead(Sensor1));
    Serial.print("   ");
  Serial.print(analogRead(Sensor2));
    Serial.print("   ");
  Serial.print(analogRead(Sensor3));
    Serial.print("   ");
  Serial.print(analogRead(Sensor4));
    Serial.print("   ");
  Serial.print(analogRead(Sensor5));
    Serial.print("   ");
  Serial.print(analogRead(Sensor6));
    Serial.print("   ");
  Serial.print(analogRead(Sensor7));
    Serial.print("   ");
  Serial.print(digitalRead(interruptor1));
    Serial.print("   ");
  Serial.print(digitalRead(interruptor2));
    Serial.print("   ");
  Serial.println(digitalRead(interruptor3));

  */


}


/*
 y esto es to...
 esto es to...
 esto es todo amigos
 */

