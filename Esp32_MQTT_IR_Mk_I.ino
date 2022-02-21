#include <WiFi.h>
#include <PubSubClient.h>
#include <IRremote.h>

//------------------------------------------------------------------------------
// Tell IRremote which Arduino pin is connected to the IR Receiver (TSOP4838)
//
int IR_RECEIVE_PIN = 13;

//+=============================================================================

//+=============================================================================
// Wi-Fi ssid and password
#define ssid       "*********"
#define password       "*******"
// Your broaker IP, in this format
IPAddress server(***, ***, *, **); 
//+=============================================================================

//+=============================================================================
// Configure the Arduino
//

bool TVon = true;
bool buttonsON = true; 
uint16_t sAddress = 0x0102;

uint8_t sRepeats = 3;

//+=============================================================================

//+=============================================================================
// Callback function, listen to the incoming message
void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Incoming message - ASCII decimal: ");
  Serial.println(*payload);

  // message handling
  switch (*payload) {
    case 114:
      break;
    case 121:
      break;
    case 103:
      break;
    default:
      break;
  }
}

WiFiClient esp32Client;
PubSubClient client(server, 1883, callback, esp32Client);

void startWifi() {

  Serial.println(F("Adafruit MQTT demo"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
}
void connectBroker() {
  // User and password configured in the broker
  if (client.connect("Arcade/test")) {
    client.publish("Arcade/test", "Ready to publish in Arcade/test"); //Esp32 publish in outTopic
    client.subscribe("Arcade/test"); //Esp32 subscribe inTopic. See callback function
    Serial.println("Broker connected!");
  }
  else {
    Serial.println("Not connected to Broker");
  }
}

void setup()
{
  //+==================================================
  
      Serial.begin(115200);   // Status message will be sent to PC at 9600 baud
  #if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
      delay(2000); // To be able to connect Serial monitor after reset or power up and before first printout
  #endif
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver, enable feedback LED, This is where the error seems to occur!!!
  //+====================================================

  startWifi();
  connectBroker();
}

void loop()
{
  client.loop();
  client.publish("Arcade/test", "test");
  Serial.println("publishing to mqtt");
  delay(500);
}
