#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <U8x8lib.h>
#include <Ultrasonic/Ultrasonic.h>

//Criação de variável na memoria RTC para contagem de boots do ESP32
RTC_DATA_ATTR uint16_t bootCount = 0;

//fator de conversão de microsegundos para segundos
#define uS_TO_S_FACTOR 1000000
//tempo que o ESP32 ficará em modo sleep (em segundos)
#define TIME_TO_SLEEP 600

// Definição do Tamanho da Lixeira (em cm)

#define TAMANHO_LIXEIRA 30.0

// Definição do pino de entrada da Tensão da bateria

#define PINO_BATERIA 2

// Definição da GPIO para controle do transistor de alimentação do sensor ultrassônico

#define PINO_TRANSISTOR 39

// Definindo pinos do sensor ultrassônico

const int trigPin = 12;
const int echoPin = 13;


// Criando objeto do tipo ultrasssonic (Sensor HC-SR04)
Ultrasonic ultrasonic(trigPin,echoPin, 12000); // (Trig PIN,Echo PIN)

// Função para determinar nível de preenchimento da Lixeira
uint8_t MedirNivelLixeira();

// Below sets the trigger for sending a new message to a gateway.
// Either or both can be enabled (in which case pressing the button will restart the timer)

#define SEND_BY_BUTTON 1  // Send a message when button "0" is pressed
#define SEND_BY_TIMER 1 // Send a message every TX_INTERVAL seconds


//**************************************************************
// Aqui são feitas as configurações do protocolo LoRaWAN
//**************************************************************


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x1A, 0xAC, 0xDA, 0x40, 0x99, 0x10, 0x36, 0x86, 0xA6, 0x7B, 0x85, 0xBC, 0x6C, 0x47, 0xA2, 0x61 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x78, 0xFE, 0xE4, 0x5B, 0x14, 0x3E, 0x3B, 0x42, 0xD2, 0x16, 0x47, 0x34, 0x5E, 0x36, 0x9A, 0xAC };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26031369;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
void do_send(osjob_t* j);
void do_send_location(osjob_t* j);

static uint8_t mydata[5];
static osjob_t sendjob;

// Pin mapping for the Heltec ESP32 V2 LoRa
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 35, 34},
};

// Configuring the OLED Display
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// If send-by-timer is enabled, define a tx interval
#ifdef SEND_BY_TIMER
#define TX_INTERVAL 60 // Message send interval in seconds
#endif


// State machine event handler
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      digitalWrite(LED_BUILTIN, LOW); // Turn off LED after send is complete
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
  // Serial.println("Going to sleep now!");
  // esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);

#ifdef SEND_BY_TIMER
      // Schedule the next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#endif
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

// Transmit data from mydata
void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    

    byte payload[3];

    long FillLevel = MedirNivelLixeira();
    long TimeSensor = ultrasonic.Timing();
    uint16_t BatteryLevelRaw = analogRead(PINO_BATERIA);
    float BatteryLevelFloat = BatteryLevelRaw * (3.27/4095) * (4.97 / 2.34) * 100;
    uint16_t BatteryLevel = BatteryLevelFloat;

    payload[0] = FillLevel;
    payload[1] = highByte(BatteryLevel);
    payload[2] = lowByte(BatteryLevel);


    Serial.print("Signal travel time: ");
    Serial.print(TimeSensor);
    Serial.println("us" );

    Serial.print("Fill Level: ");
    Serial.print(FillLevel);
    Serial.println(" % " );

    Serial.print("Batery Level: ");
    Serial.println(BatteryLevel);

    Serial.print("Packet Size: ");
    Serial.print(sizeof(payload));
    Serial.println("bytes");

    Serial.print("Data been sent: ");
    for(int i=0;i < sizeof(payload);i++)
    {
      Serial.print(payload[i]);
      Serial.print(" ");
    }
    Serial.println(" ");



    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED while sending
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println(F("Packet queued"));

  }
}


// Setup Funcion

void setup() {

  // put your setup code here, to run once:
  // definição do PINO_BATERIA como entrada

  pinMode(PINO_BATERIA,INPUT);

  // definindo pino de controle da alimentação do Sensor Ultrassônico

  pinMode(PINO_TRANSISTOR,OUTPUT);

  // Desligando alimentação do sensor

  digitalWrite(PINO_TRANSISTOR,LOW);
   

  // Início da Serial
  Serial.begin(115200);
  Serial.println("Serial Port Connected");

  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 0, "Teste Henrique");



  // bootCount++;
  // Serial.print("Booting order: ");
  // Serial.println(bootCount);

  // Set button pin as input
#ifdef SEND_BY_BUTTON
  pinMode(0, INPUT);
#endif
  // Configure built-in LED -- will illuminate when sending
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

#if defined(CFG_eu868) // EU channel setup
  // Set up the channel used by the Things Network and compatible with
  // our gateway.
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  for (int b = 1; b < 8; ++b) {
    LMIC_disableChannel(b);
  }

#elif defined(CFG_us915) // US channel setup
  // Instead of using selectSubBand, which will cycle through a sub-band of 8
  // channels. We'll configure the device to only use one frequency.
  // First disfable all sub-bands
  for (int b = 0; b < 8; ++b) {
    LMIC_disableSubBand(b);
  }
  // Then enable the channel(s) you want to use
  //LMIC_enableChannel(8); // 903.9 MHz
  LMIC_enableChannel(17);
#endif

  //

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job -- Transmit a message on begin
   do_send(&sendjob);

   delay(2000);

  // Chama a função para colocar o ESP32 em Deep Sleep

   esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  
}


void loop() {
  // put your main code here, to run repeatedly:

 os_runloop_once();

 

#ifdef SEND_BY_BUTTON
  if (digitalRead(0) == LOW) {
    while (digitalRead(0) == LOW) ;
    do_send(&sendjob);

    // Serial.println("Testando leitura do nivel de lixo!");
    // uint8_t x = MedirNivelLixeira();
    // Serial.print("O nivel medido é ");
    // Serial.print(x);
    // Serial.println(" %% !");
  }
#endif




}


uint8_t MedirNivelLixeira()
{
  uint16_t altura_temp = 0;
  uint16_t altura_media = 0;
  float nivel;
  uint8_t resultado;

  // ativando a alimentação do sensor ultrassonico

  digitalWrite(PINO_TRANSISTOR, HIGH);
  delayMicroseconds(10);

  Serial.println("Iniciando medição");
  Serial.println(" ");
 for (int i=0; i < 10; i++)
  {
    altura_temp += ultrasonic.Ranging(CM);
    delay(10); 
  }
  altura_media = altura_temp/10;
  // Serial.print("Altura medida: ");
  // Serial.println(altura_media);
  
  nivel = 1 - altura_media/TAMANHO_LIXEIRA;
  // Serial.print("Nivel medido: ");
  // Serial.print(nivel);
  // Serial.println(" %");

  if (nivel >= 0 && nivel < 0.1)
  {
    resultado = 1;
  }
  else if(nivel >= 0.1 && nivel < 0.2 )
  { 
    resultado = 10;
  } 
  else if(nivel >= 0.2 && nivel < 0.3 )
  { 
    resultado = 20;
  }
  else if(nivel >= 0.3 && nivel < 0.4 )
  { 
    resultado = 30;
  }
  else if(nivel >= 0.4 && nivel < 0.5 )
  { 
    resultado = 40;
  }
  else if(nivel >= 0.5 && nivel < 0.6 )
  { 
    resultado = 50;
  }
  else if(nivel >= 0.6 && nivel < 0.7 )
  { 
    resultado = 60;
  }
  else if(nivel >= 0.7 && nivel < 0.8 )
  { 
    resultado = 70;
  }
 else if(nivel >= 0.8 && nivel < 0.9 )
  { 
    resultado = 80;
  }
  else if(nivel >= 0.9 && nivel < 0.95)
  { 
    resultado = 90;
  }
  else if(nivel >= 0.95) 
  { 
    resultado = 100;
  }
  else
  {
    resultado = 0;
  }
  
  // desligando alimentação do sensor ultrassonico

  digitalWrite(PINO_TRANSISTOR, LOW);
  delayMicroseconds(10);

  return(resultado);

};



// // Transmit data from mydata
// void do_send_location(osjob_t* j) {
//   // Check if there is not a current TX/RX job running
//   if (LMIC.opmode & OP_TXRXPEND) {
//     Serial.println(F("OP_TXRXPEND, not sending"));
//   } else {
    
//     int32_t latitude_binary = LAT * 1000000;
//     int32_t longitude_binary = LNG * 1000000;

//     Serial.print("Latitude: ");
//     Serial.println(LAT, 6);
//     Serial.print("Longitude: ");
//     Serial.println(LNG, 6);

//     Serial.println("Converted to int32 ");
//     Serial.println(latitude_binary);
//     Serial.println(longitude_binary);
  

//     byte payload[9];
//     payload[0] = 2;

//     payload[1] = (latitude_binary >> 24) & 0xFF;
//     payload[2] = (latitude_binary >> 16) & 0xFF;
//     payload[3] = (latitude_binary >> 8) & 0xFF;
//     payload[4] = latitude_binary & 0xFF;

//     payload[5] = (longitude_binary >> 24) & 0xFF;
//     payload[6] = (longitude_binary >> 16) & 0xFF;
//     payload[7] = (longitude_binary >> 8) & 0xFF;
//     payload[8] = longitude_binary & 0xFF;

//     Serial.print("Data been sent: ");

//     for(int i =0; i < sizeof(payload); i++)
//     {
//       Serial.print(payload[i], HEX);
//       Serial.print(" ");
//     }

//     Serial.println(" ");



//     digitalWrite(LED_BUILTIN, HIGH); // Turn on LED while sending
//     // Prepare upstream data transmission at the next possible time.
//     LMIC_setTxData2(1, payload, sizeof(payload), 0);
//     Serial.println(F("Location Packet queued"));

//   }
// }