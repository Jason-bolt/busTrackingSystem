// This example shows how to connect to Cayenne using an ESP32 and send/receive sample data.
// Make sure you install the ESP32 Board Package and select the correct ESP32 board before compiling.
// To install the ESP32 Board Package follow the instructions here: https://github.com/espressif/arduino-esp32/blob/master/README.md#installation-instructions.

//#define CAYENNE_DEBUG
#define CAYENNE_PRINT Serial
#include <CayenneMQTTESP32.h>


#define LATITUDE_VIRTUAL_CHANNEL 1
#define LONGITUDE_VIRTUAL_CHANNEL 2
#define MPS_VIRTUAL_CHANNEL 3
#define KMPH_VIRTUAL_CHANNEL 4




// Your GPRS credentials (leave empty, if not needed)
const char apn[]      = "internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = "Vodafone GH"; // GPRS User
const char gprsPass[] = ""; // GPRS Password

// SIM card PIN (leave empty, if not defined)
const char simPIN[]   = "";

// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

#define uS_TO_S_FACTOR 1000000UL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */
//#define TIME_TO_SLEEP  1 

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}










//// WiFi network info.
//char ssid[] = "iotdevlab+virus";
//char wifiPassword[] = "m,./@1234";
//WiFiClient client;

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "50457b00-f653-11e9-a38a-d57172a4b4d4";
char password[] = "d02f0fa606e75ca84124ef4564b50c5c292b6886";
char clientID[] = "7d58caa0-7948-11ec-8da3-474359af83d7";

unsigned long lastMillis = 0;

void setup() {
  Serial.begin(9600);
//  WiFi.begin(ssid, wifiPassword);
//  
//      while (WiFi.status() != WL_CONNECTED) {
//          delay(500);
//      }

  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);

  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  modem.restart();
  // use modem.init() if you don't need the complete restart

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  
  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  connectGSM();
      
  Cayenne.begin(username, password, clientID);
}





void loop() {
  Cayenne.loop();
}





// Default function for sending sensor data at intervals to Cayenne.
// You can also use functions for specific channels, e.g CAYENNE_OUT(1) for sending channel 1 data.
CAYENNE_OUT_DEFAULT()
{
//  Latitude
  Cayenne.virtualWrite(LATITUDE_VIRTUAL_CHANNEL, millis());
  Cayenne.virtualWrite(LONGITUDE_VIRTUAL_CHANNEL, millis());
  Cayenne.virtualWrite(MPS_VIRTUAL_CHANNEL, millis());
  Cayenne.virtualWrite(KMPH_VIRTUAL_CHANNEL, millis());
  
}




bool connectGSM(){
  Serial.print("Connecting to APN: ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" fail");
  }
  else {
    Serial.println(" OK");
  }
}
