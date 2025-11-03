#define BLYNK_TEMPLATE_ID "TMPL2_Di9Z7gr"
#define BLYNK_TEMPLATE_NAME "Street Lamp"
#define BLYNK_AUTH_TOKEN "McbtRAUfSQE5QwvbC2yEoYX6r6YNE0Ud"

#define BLYNK_PRINT Serial

// ******************* MODEM SIM7080 **************************
#define TINY_GSM_MODEM_SIM7080

#include <Arduino.h>
#include <Wire.h>
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>

// ********** CONFIGURACION DE COMUNICACIONES SERIALES ********
#define SerialMon Serial
#define SerialAT Serial2
#define TINY_GSM_DEBUG SerialMon


// ************ CONFIGURACION DE PINOUT MIKROBUS ***************
#define MIKROBUS_AN   4
#define MIKROBUS_RST  15
#define MIKROBUS_CS   6
#define MIKROBUS_SCK  8
#define MIKROBUS_MISO 18
#define MIKROBUS_MOSI 17
#define MIKROBUS_PWM  5
#define MIKROBUS_INT  7
#define MIKROBUS_RX   9
#define MIKROBUS_TX   10
#define MIKROBUS_SCL  13
#define MIKROBUS_SDA  12

#define BOARD_LED 16

#define UPDATE_BUTTON 0
#define PIN_MODEM_PK MIKROBUS_INT

static BlynkTimer scheduler;
static TinyGsm modem(SerialAT);


const char apn[] = "";
const char user[] = "";
const char pass[] = "";

const char domain[] = "ny3.blynk.cloud";
const char auth[] = BLYNK_AUTH_TOKEN;

// Dirección I2C del XN11 (11 decimal = 0x0B)
const uint8_t XN11_ADDR = 0x0B;

// Dirección I2C del XN01 Digital Input (1 decimal = 0x01)
const uint8_t XN01_ADDR = 0x01;

uint16_t readXN04Luminosity( ){

    uint16_t lux;
    // XN04
    Wire.beginTransmission( 4 );
    // Registro de lux
    Wire.write( 0x03 );
    Wire.endTransmission();

    Wire.requestFrom( 4, 2 );
    
    lux = ( (Wire.read() << 8) | Wire.read() );

    return lux;

}
// Devuelve 0 si OK, otro valor si hay error I2C
uint8_t writeXN11(uint8_t relay, uint8_t stat)
{
  uint8_t reg = 0x00;
  uint8_t data = stat ? 0x01 : 0x00;

  switch (relay) {
    case 1:
    case 2:
      reg = relay;      // registro 0x01 u 0x02
      break;
    default:
      return 4;         // código “otros errores” típico de Wire
  }

  Wire.beginTransmission(XN11_ADDR);
  Wire.write(reg);      // seleccionar canal/registro
  Wire.write(data);     // 0x01 = ON, 0x00 = OFF
  return Wire.endTransmission(); // 0=éxito
}

// Función para leer entradas digitales del X-NODE
uint8_t readDigitalInputs()
{
  Wire.beginTransmission(XN01_ADDR);
  Wire.write(0x01); // Selecciona registro de entradas
  Wire.endTransmission();

  Wire.requestFrom(XN01_ADDR, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0; // Si falla, retorna 0
}

void updateTemperature()
{
    Blynk.virtualWrite(V2, readXN04Temperature());
}

void updateHumidity()
{
    Blynk.virtualWrite(V3, readXN04Humidity());
}

float readXN04Humidity( ){

    float humidity;
    uint16_t humidity_int;
    // XN04
    Wire.beginTransmission( 4 );
    // Registro de humidity
    Wire.write( 0x02 );
    Wire.endTransmission();

    Wire.requestFrom( 4, 2 );
    
    humidity_int = ( (Wire.read() << 8) | Wire.read() );
    humidity = humidity_int/100.0f;

    return humidity;

}

float readXN04Temperature()
{

    float temperature;
    uint16_t temperature_int;
    // XN04
    Wire.beginTransmission(4);
    // Registro de temperature
    Wire.write(0x01);
    Wire.endTransmission();

    Wire.requestFrom(4, 2);

    temperature_int = ((Wire.read() << 8) | Wire.read());
    temperature = temperature_int / 100.0f;

    return temperature;
}
void writeMode(uint8_t state) {
  if (state) {
    lux = readXN04Luminosity(  );
    // 2. Control de relay 1 según luminosidad
    if (lux < 500) {
      writeXN11(1, HIGH); // Enciende lámpara
    } else {
      writeXN11(1, LOW);  // Apaga lámpara
    }
  } else {
     
  }


void writeLamp(uint8_t state) {
  if (state) {
    writeXN11(1, HIGH); // Enciende lámpara
  } else {
    writeXN11(1, LOW);  // Apaga lámpara
     delay(1500); 
  }
}

void setup() {
  Serial.begin(115200);

  // I2C en pines MikroBUS
  Wire.setPins(MIKROBUS_SDA, MIKROBUS_SCL);
  Wire.begin();

  // Apaga ambos relés al inicio
  writeXN11(1, LOW);
  writeXN11(2, LOW);

   SerialMon.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MIKROBUS_RX, MIKROBUS_TX);
    

    // Inicialización de I/O
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LOW);
    pinMode(PIN_MODEM_PK, OUTPUT);

    // Reinicio del modem LTE por hardware
    digitalWrite(PIN_MODEM_PK, HIGH);
    delay(3000);
    digitalWrite(PIN_MODEM_PK, LOW);

    // Configuración inicial del modem LTE
    SerialMon.println("Iniciando modem LTE...");
    modem.restart();
    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem: ");
    SerialMon.println(modemInfo);

    // Iniciar comunicación con Blynk.Cloud
    Blynk.begin(auth, modem, apn, user, pass, domain);

    // Una vez inicializada la comunicación con el servidor
    // éste se puede interrumpir y reanudar utilizando las
    // funciones:

    // Interrumpir la conexión con el servidor de Blynk
    // Blynk.disconnect();

    // Reanudar la conexión con el servidor de Blynk
    // Blynk.connect();

    // Inicia los temporizadores
    scheduler.setInterval(10000UL, updateTemperature);
    scheduler.setInterval(10000UL, updateHumidity);
}

void loop() {

  Blynk.run();
  scheduler.run();
  // 1. Leer luminosidad ambiental
  int luzRaw = analogRead(MIKROBUS_AN);  
  // Supongamos que 0-1023 → 0-1000 lx
  float lux = (luzRaw / 1023.0) * 1000;



  lux = readXN04Luminosity(  );
  // 2. Control de relay 1 según luminosidad
  if (lux < 500) {
    writeXN11(1, HIGH); // Enciende lámpara
  } else {
    writeXN11(1, LOW);  // Apaga lámpara
  }

  // 3. Leer botón de pánico en X-NODE
  uint8_t inputs = readDigitalInputs();
  Serial.print("Entradas digitales: ");
  Serial.println(inputs, BIN);

  const uint8_t PANIC_MASK = (1 << 6);
  // Supongamos que el botón de pánico está en la entrada 0 (LSB)
  if (inputs & PANIC_MASK) {
    Serial.println("¡Botón de pánico presionado!");
    writeXN11(2, HIGH); // Enciende buzzer
    delay(15000);        // Mantener 15 segundos
    writeXN11(2, LOW);  // Apaga buzzer
  }

  delay(500); // Pequeño retardo para no saturar el bus I2C
}


BLYNK_CONNECTED()
{
    Serial.println("Conectado");
    // Obtener últimos valores del servidor
    Blynk.syncVirtual(V0, V4, V1);
}

BLYNK_DISCONNECTED()
{
    Serial.println("Desconectado");
}

BLYNK_WRITE(V1) {
  int lampState = param.asInt();  // 0 = apagada, 1 = encendida
  writeLamp(lampState); 
  Serial.println("Prender");      // actualiza físicamente la salida
}

BLYNK_WRITE(V0) {
  int modeState = param.asInt();  // 0 = apagada, 1 = encendida
  writeMode(modeState); 
       
}

BLYNK_WRITE(V2)
{
    double treshold = param.asDouble();

    if ( readXN04Temperature() < treshold ){
        Serial.println( "Temperatura actual menor al umbral" );
    } else {
        Serial.println( "Temperatura actual mayor al umbral" );
    }
}