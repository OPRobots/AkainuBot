#include <PIDfromBT.h>

//Pines de los motores
int MotorIalante = 5;
int MotorIatras = 4;
int MotorDalante = 7;
int MotorDatras = 8;
int PWM_MotorD = 9;
int PWM_MotorI = 3;

int estanbay = 6;

//Pines de los Sensores lellendolos de izquierda a derecha viendo el robot de la bateria a los sensores
int sensores[] = {A7, A6, A3, A2, A1};

//Pines de los interruptores para diferentes modos de velocidad
int interruptor1 = 10;
int interruptor2 = 11;
int interruptor3 = 12;

//Variables calibracion
int negros[] = {0, 0, 0, 0, 0, 0, 0};
int blancos[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};
int umbrales[] = {0, 0, 0, 0, 0, 0, 0};

int pesos[] = {1, 2, 3, 4, 5, 6, 7};
// este array es un array intermediario para hacer operaciones no tiene relevancia en el programa
int lectura[] = {0, 0, 0, 0, 0, 0, 0};

//Variables PID    SIN MILLIS
float kp = 20;   //1.65
float kd = 5
;    //90
float ki = 0;     //-0.24
int posicion_linea = 0;
int posicion_anterior = 0;
int velocidad = 0;
int integral = 0;
int correccion = 0;

//Variables calculo analogico

int posicion = 0;
double media = 0;
double suma = 0;
int sensores_detectando = 0;
int posicionMax = 1020; //posible calcular comprovar valores
int posicionMin = -1020; //     ""






//Variables barias
bool calibracion = true;
bool arranque = true;
int i = 0;
int recivido = 0;
long millisAnterior = 0;
long millisInicio = 0;
int umbral_blanco = 140;
int umbral_negro = 30;



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
    pinMode(sensores[i], INPUT);
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

  // ========================================================
  //realizamos la calibracion inicial obteniendo los minimos y maximos leidos por cada sensor durante 5 seg y calculamos los umbrales
  // ========================================================

  if (calibracion == true) {
    digitalWrite(13, HIGH);
    millisAnterior = millis();
    do {

      // lectura de los sensores
      for (i = 0; i < 7; i++) {
        lectura[i] = analogRead(sensores[i]);
        //lectura[i] = constrain (map ((1023 - analogRead(sensores[i])), 0, 1023, 0, 255), 0, 255);

      }

      // registramos los valores maximos y minimos en negros y blancos respectivamente
      for (i = 0; i < 5; i++) {
        if (lectura[i] > negros[i]) {
          negros[i] = lectura[i];
        }
        if (lectura[i] < blancos[i]) {
          blancos[i] = lectura[i];
        }
      }

    } while ((millis() - millisAnterior) <= 5000);

    // Calculamos los umbrales para cada sensor
    for (i = 0; i < 5; i++) {
      umbrales[i] = ((negros[i] + blancos[i]) / 2);
    }



    for (i = 0; i < 5; i++) {
      Serial.print(umbrales[i]);
      Serial.print("   ");
    }
    Serial.println("   ");
    calibracion = false;
    digitalWrite(13, LOW);
  }







  if (digitalRead(2) == false && arranque == true) {
    arranque = false;
    millisInicio = millis();
  } else if (millis() >= (millisInicio + 5000) && arranque == false && millisInicio > 0) {

    if (digitalRead(10) == false) {
      vel = 65;
      kp = 2;   //1.65
      kd = 90;    //90
      ki = 0;
    }



    //  ========================================================
    // Calculo del analogico de desplazamiento del array de sensores
    //  ========================================================




    //cada 20 milisegundos lee los sensores y llama a la funcion que ejecuta el calculo para hacerlo de manera periodica

    //  if (millis() % 1 == 0) {




    sensores_detectando = 0;
    for (int i = 0; i < 5; i++) {
      lectura[i] = map ((analogRead(sensores[i])), blancos[i], negros[i], 0, 255);

      if (lectura[i] >= 200) {
        lectura[i] = 255;
      } else if (sensores[i] < 150) {
        sensores_detectando++;
        lectura[i] = 0;
      }

     // Serial.print(lectura[i]);
     // Serial.print(" ");
    }
  //  Serial.println(" ");




    media = 0;
    suma = 0;
    for (i = 0; i < 5; i++) {
      media = media + (lectura[i] * pesos[i] * 10);
      suma = suma + lectura[i];
    }




    if (sensores_detectando > 0) {
      posicion = media / suma;
      posicion_linea = posicion - (((5 + 1) * 10) / 2);
    }
    else if (posicion_linea > 0) {
      posicion_linea = posicionMax;
    }
    else if (posicion_linea < 0) {
      Serial.println("no sensores iz");
      posicion_linea = posicionMin;
    }

    


    Serial.println(posicion_linea);


    // ========================================================
    //                        Calculo del PID
    //  ========================================================


    velocidad = posicion_linea - posicion_anterior;
    integral = integral + (posicion_linea / 10);
    //integral= constrain(integral,-500,500);

    correccion = ((kp * posicion_linea) + (kd * velocidad) + (ki * integral));

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
}


/*
  y esto es to...
  esto es to...
  esto es todo amigos
*/

