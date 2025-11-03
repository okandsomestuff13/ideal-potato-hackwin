#include <QTRSensors.h>

// --------- Pines QTR-8A (analógico) ---------
#define S1 A0
#define S2 A1
#define S3 A2
#define S4 A3
#define S5 A4
#define S6 A5
#define S7 A6
#define S8 A7
#define IR_PIN 11        // Control de emisores IR

// --------- Pines TB6612FNG ---------
#define PWMA 3           // motor Izquierdo PWM
#define AIN1 5
#define AIN2 4
#define PWMB 9           // motor Derecho PWM
#define BIN1 7
#define BIN2 8
#define STBY 6

// --------- Botón y LED ---------
#define BUTTON 12
#define LED    13

// --------- Sensores / Librería ---------
QTRSensors qtr;
unsigned int sensorValues[8];

// --------- PID ---------
float Kp = 0.20f;
float Ki = 0.00f;
float Kd = 0.50f;

int   baseSpeed = 200;
long  integral  = 0;
int   lastError = 0;

// --------- Utilidad motores ---------
void setMotors(int leftSpeed, int rightSpeed) {
  // IZQUIERDO (A)
  if (leftSpeed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); leftSpeed = -leftSpeed; }
  analogWrite(PWMA, constrain(leftSpeed, 0, 255));

  // DERECHO (B)
  if (rightSpeed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); rightSpeed = -rightSpeed; }
  analogWrite(PWMB, constrain(rightSpeed, 0, 255));
}

void brakeMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void setup() {
  Serial.begin(9600);

  // Pines motores
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH); // habilita TB6612

  // Botón/LED
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT); digitalWrite(LED, LOW);

  // QTR-8A analógico
  unsigned char pins[8] = { S1, S2, S3, S4, S5, S6, S7, S8 };
  qtr.setTypeAnalog();
  qtr.setSensorPins(pins, 8);
  qtr.setEmitterPin(IR_PIN);

  // --------- Calibración guiada ---------
  Serial.println(F("Presiona el BOTON para iniciar calibracion..."));
  while (digitalRead(BUTTON) == HIGH) { /* espera */ }

  Serial.println(F("Calibrando (moviéndose para ver blanco/negro)..."));
  digitalWrite(LED, HIGH);

  for (int i = 0; i < 240; i++) {
    if ((i / 20) % 2 == 0) setMotors(90, -90);
    else                   setMotors(-90, 90);
    qtr.calibrate();
    delay(10);
  }
  brakeMotors();
  digitalWrite(LED, LOW);
  Serial.println(F("Calibracion finalizada!"));

  for (int i = 0; i < 3; i++) { digitalWrite(LED, HIGH); delay(250); digitalWrite(LED, LOW); delay(250); }

  Serial.println(F("Coloca el robot sobre la linea BLANCA y presiona BOTON para iniciar..."));
  while (digitalRead(BUTTON) == HIGH) { /* espera */ }
  delay(300);
  digitalWrite(LED, HIGH); // LED encendido = modo seguimiento
}

void loop() {
  // Para linea BLANCA sobre fondo negro:
  // readLineWhite() devuelve [0..7000] (centro ≈ 3500) y llena sensorValues calibrados (0..1000)
  int position = qtr.readLineWhite(sensorValues);

  // Suma total para detectar pérdida de línea (en negro puro la suma cae)
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += sensorValues[i];

  int error = position - 3500;

  // PID
  integral += error;
  integral = constrain(integral, -20000L, 20000L);
  int derivative = error - lastError;
  int correction = (int)(Kp * error + Ki * integral + Kd * derivative);
  lastError = error;

  int left  = baseSpeed - correction;
  int right = baseSpeed + correction;

  // Recuperación si la suma es muy baja (estamos en negro, sin ver la línea blanca)
  if (sum < 800) { // ajusta este umbral según tu pista/iluminación
    if (lastError >= 0) setMotors(60, -60);   // buscar a la derecha
    else                setMotors(-60, 60);   // buscar a la izquierda
    delay(20);
    return;
  }

  setMotors(constrain(left, -255, 255), constrain(right, -255, 255));

}
