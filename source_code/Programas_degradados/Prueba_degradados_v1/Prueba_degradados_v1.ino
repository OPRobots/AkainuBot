
#include <PIDfromBT.h>

//============================
//variables pines//
//============================

int sensor[] = {A7, A6, A5, A5, A4, A3, A2, A1};

int MotorIalante = 4;
int MotorIatras = 5;
int MotorDalante = 8;
int MotorDatras = 7;
int PWM_MotorD = 9;
int PWM_MotorI = 3;
int estanbay = 6;

int interruptor1 = 10;
int interruptor2 = 11;
int interruptor3 = 12;

//============================
//variables calibración//
//============================

int valores[] = {0, 0, 0, 0, 0, 0, 0};
int valores_min[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};
int valores_max[] = {0, 0, 0, 0, 0, 0, 0};
float media = 0;
float media_min = 1000;
float media_max = 0;

bool calibracion1_ok = false;
bool calibracion2_ok = false;

//============================
//variables PID//
//============================

float kp = 2;
float kd = 0;
float ki = 0;

int posicion = 0;  //posicion del robot sobre la pista (de -1000 a 1000)
int posicion_anterior = 0;
int integral = 0;
int derivatiba = 0;
int correccion = 0;
int ideal = -150;    //valor ideal del PID (de -250 a 250)


//============================
//variables varias//
//============================
int i = 0;
bool arranque = true;
long millisAnterior = 0;
long millisInicio = 0;


int velD = 0;
int velI = 0;
int vel = 0;    //150

PIDfromBT pid_calibrate(&kp, &ki, &kd, &vel, DEBUG);


void setup() {
  pinMode(MotorIalante, OUTPUT);
  pinMode(MotorIatras, OUTPUT);
  pinMode(MotorDalante, OUTPUT);
  pinMode(MotorDatras, OUTPUT);


  for (i = 0; i < 7; i++) {
    pinMode(sensor[i], INPUT);
  }

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


  // ========================================================
  //Comunicacion del bluethooth para calibracion del PID
  // ========================================================

  pid_calibrate.update();


  //============================
  //calibración//
  //============================

  if (calibracion1_ok == false) {

    digitalWrite(13, HIGH);

    millisAnterior = millis();
    do {

      for (i = 0; i < 7; i++) {
        valores[i] = analogRead(sensor[i]);
      }


      for (i = 0; i < 7; i++) {
        if (valores[i] < valores_min[i]) {
          valores_min[i] = valores[i];
        }
        if (valores[i] > valores_max[i]) {
          valores_max[i] = valores [i];
        }
      }

    } while ((millis() - millisAnterior) <= 5000);

    for (i = 0; i < 7; i++) {
      Serial.print(valores_min[i]);
      Serial.print("   ");
    }
    for (i = 0; i < 7; i++) {
      Serial.print(valores_max[i]);
      Serial.print("   ");
    }
    Serial.println("   ");
    calibracion1_ok = true;
    digitalWrite(13, LOW);
    delay(700);
  }



  if (calibracion1_ok == true && calibracion2_ok == false) {

    digitalWrite(13, HIGH);
    millisAnterior = millis();

    do {

      for (i = 0; i < 7; i++) {
        media = media + map(analogRead(sensor[i]), valores_min[i], valores_max[i], 0, 250);
      }
      media = media / 7;

      if (media < media_min) {
        media_min = media;
      } else if (media > media_max) {
        media_max = media;
      }


    } while ((millis() - millisAnterior) <= 5000);


    Serial.print(media_min);
    Serial.print("   ");
    Serial.print(media_max);
    Serial.println("   ");

    calibracion2_ok = true;
    digitalWrite(13, LOW);
  }





  //============================
  //programacion interruptores//
  //============================

  if (digitalRead(10) == false && arranque == true) {
    arranque = false;
    millisInicio = millis();
  } else if (millis() >= (millisInicio + 500) && arranque == false && millisInicio > 0) {

    if (digitalRead(11) == false) {
      vel = 0;
      kp = 2;   //1.65
      kd = 0;    //90
      ki = 0;
    }
    if (digitalRead(12) == false) {
      vel = 35;
      kp = 2;   //1.65
      kd = 0;    //90
      ki = 0;
    }



    //============================
    //Calculo posicion robot//
    //============================

    for (i = 0; i < 7; i++) {
      valores[i] = map(analogRead(sensor[i]), valores_min[i], valores_max[i], 0, 250);
    }
    media = 0;
    for (i = 0; i < 7; i++) {
      media = media + valores[i];
    }
    posicion = map((media / 7), media_min, media_max, 0, 250);

    if (((valores[7]+valores[6]) > (valores[1]+valores[2]))) {     //||posicion_anterior < -5
      posicion = posicion * -1;

    }
    posicion = posicion + ideal;

Serial.println(posicion);



    // ========================================================
    //                        Calculo del PID
    //  ========================================================


    derivatiba = posicion - posicion_anterior;
    integral = integral + (posicion / 10);

    correccion = ((kp * posicion) + (kd * derivatiba) + (ki * integral));

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

    // }


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

    // ========================================================
    //       Asignacion de velocidades a los motores
    // ========================================================


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

    // ========================================================
    //               COMPROBACIÓN DE SENSORES
    // ========================================================

  /*  for(i=0;i<7;i++){
      Serial.print( map(analogRead(sensor[i]), valores_min[i], valores_max[i], 0, 250));
      Serial.print(" ");
    }
      Serial.println(" ");
*/
  }
}


/*
  y esto es to...
  esto es to...
  esto es todo amigos
*/

