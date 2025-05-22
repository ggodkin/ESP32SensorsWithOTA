#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <esp_sleep.h>
#include <credentials.h> // Include the new credentials file
#include <esp_chip_info.h> // Include for chip information
#include <esp_flash.h>     // Include for flash information
#include <esp_wifi.h>      // Include for esp_wifi_start() and esp_wifi_set_channel()
#include <esp_event.h>     // Include for ESP-IDF event handling
#include <esp_wifi_types.h> // Include for WIFI_EVENT and WIFI_EVENT_STA_START, wifi_interface_t
#include <esp_system.h>    // Include for esp_get_free_heap_size() (optional, for debugging)
#include <string.h>      // Include for memset
#include <driver/adc.h>    // Include for ADC attenuation settings (defines adc_attenuation_t)
#include <driver/gpio.h>   // Include for GPIO specific functions
#include <esp_intr_alloc.h> // Include for interrupt allocation

// --- Includes for Sensor Task (when GPIO 23 is HIGH) ---
#include <esp_now.h>       // ESP-NOW library for ESP32
#include <OneWire.h>       // OneWire library for DS18B20
#include <DallasTemperature.h> // Dallas Temperature library for DS18B20

// --- Configuration ---
// Define DEBUG level:
// 0: No debug output, LED_PIN 99, sleepTime 15min
// 1: Serial debug output, LED_BUILTIN, sleepTime 2min
// 2: Serial1 debug output, LED_PIN 99, sleepTime 2min
// 4: Serial debug output, LED_BUILTIN, sleepTime 30sec
#define DEBUG 0

#if DEBUG == 2
  #define debugln(x) Serial1.println(x)
  #define debug(x) Serial1.print(x)
  #define debugBegin(x) Serial1.begin(x)
  #define serial_port Serial1 // Renamed to avoid conflict with boolean 'true'
  #define LED_PIN_MACRO     99
  //Deep Sleep time in microseconds
  #define sleepTime 120e6   //2min
#elif DEBUG == 1
  #define debugln(x) Serial.println(x)
  #define debug(x) Serial.print(x)
  #define debugBegin(x) Serial.begin(x)
  #define serial_port Serial // Renamed to avoid conflict with boolean 'true'
  #define LED_PIN_MACRO     LED_BUILTIN
  //Deep Sleep time in microseconds
  #define sleepTime 120e6   //2min
#elif DEBUG == 4
  #define debugln(x) Serial.println(x)
  #define debug(x) Serial.print(x)
  #define debugBegin(x) Serial.begin(x)
  #define serial_port Serial // Renamed to avoid conflict with boolean 'true'
  #define LED_PIN_MACRO     LED_BUILTIN
  //Deep Sleep time in microseconds
  #define sleepTime 30e6    //30 sec
#else // DEBUG == 0 or undefined
  #define debugln(x)
  #define debug(x)
  #define debugBegin(x)
  #define serial_port // No serial port defined for no debug
  #define LED_PIN_MACRO     LED_BUILTIN
  //Deep Sleep time in microseconds
  #define sleepTime 900e6   //15min
#endif


// GPIO pin to check for OTA activation.
// Connect this pin to GND at boot to activate OTA.
// Leave unconnected or connect to VCC for the Sensor Task and deep sleep.
const int OTAPin = 23;

// LED Pin (change to your board's built-in LED or desired GPIO)
// GPIO 2 is a common pin for the built-in LED on many ESP32 development boards.
// Using the macro defined above.
#ifndef LED_BUILTIN // Define LED_BUILTIN if not already defined for your board
#define LED_BUILTIN 2
#endif
const int LED_PIN = LED_PIN_MACRO;


// --- Pin Definitions for Sensor Task (MAP THESE TO YOUR ESP32 GPIOs) ---
// GPIO for powering the DS18B20 sensor
const int TEMP_POWER = 16; // Example: Map to an available ESP32 GPIO
// GPIO for grounding the ADC (if used for voltage measurement)
// If this pin is used to enable/disable a voltage divider to ground,
// setting it HIGH (output) after measurement might be what you want
// to open the circuit and save power.
const int ADC_GND = 17;    // Example: Map to an available ESP32 GPIO
// GPIO where the DS18B20 data line is connected to
const int TEMP_DATA = 4;   // Example: Map to an available ESP32 GPIO
// Analog pin for battery voltage measurement (MAP TO ESP32 ADC GPIO)
const int ANALOG_PIN = 34; // Example: Map to an ESP32 ADC GPIO (e.g., GPIO34 is ADC1_CHANNEL6)

// GPIO for the tipping bucket rain gauge interrupt (MUST BE an RTC GPIO for deep sleep wake-up)
const int RAIN_GAUGE_PIN = 14; // GPIO14 is an RTC GPIO

// Deep Sleep time in microseconds (adjust as needed)
// Using the macro defined above.
const uint64_t periodicSleepTime = sleepTime;

// WiFi Channel for ESP-NOW (Both sender and receiver should be on the same channel)
const int ESPNOW_CHANNEL = 1; // Example: Use channel 1

// Minimum interval between rain gauge tips being processed (in milliseconds)
// This helps debounce and prevent rapid re-triggering of messages from a single event.
const unsigned long MIN_RAIN_TIP_INTERVAL_MS = 1000; // 1 second debounce for messages


// --- Web Server and OTA Objects ---
// WebServer instance on port 80
WebServer server(80);

// --- Sensor Task Variables and Objects ---
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_DATA);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// REPLACE WITH RECEIVER MAC Address for ESP-NOW
uint8_t broadcastAddress[] = {0x2C, 0xF4, 0x32, 0x20, 0x5D, 0x1C}; // Example MAC Address
uint8_t tempDeviceAddress[8]; // For storing DS18B20 sensor address
String strMAC;

// DS18B20 count - Made global for EspWifiEventHandler access
int devCount = 0;

// Structure example to send data via ESP-NOW
// Must match the receiver structure
typedef struct struct_message {
  char a[240];
} struct_message;

// Create a struct_message called myData
struct_message myData;
String sendMsg;

// Variable to count rain gauge tips
// RTC_DATA_ATTR ensures this variable retains its value during deep sleep
RTC_DATA_ATTR volatile int savedRainGaugeTips = 0;
// RTC_DATA_ATTR to store the last time a rain tip was processed (for software debouncing)
RTC_DATA_ATTR unsigned long lastProcessedRainTipMillis = 0;


// Flag to indicate if sensor data has been sent via ESP-NOW
static bool sensor_data_sent = false;

// Global variable to store wakeup cause
esp_sleep_wakeup_cause_t global_wakeup_cause;


// --- ESP-NOW Send Callback ---
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
  debug("OnDataSent Callback - Last Packet Send Status: ");
  if (sendStatus == ESP_NOW_SEND_SUCCESS) {
    debugln("Delivery success");
  } else {
    debugln("Delivery fail");
    digitalWrite(LED_PIN, HIGH); // Indicate failure with LED
  }
  debug("On Data Sent Time (ms): ");
  debugln(millis());
#if DEBUG > 0
  serial_port.flush(); // Flush serial buffer
#endif
  debugln("OnDataSent Callback - Adding small delay after send status...");
  delay(20); // Add a small delay (adjust if needed)
  debugln("OnDataSent Callback - Delay finished.");

  // Data has been sent (or failed), set flag to allow deep sleep
  sensor_data_sent = true;
}

// --- Helper function to convert DS18B20 address to string ---
String mac2string() {
  char str[3]; // Need 3 chars for two hex digits and null terminator
  String strMAC = "MAC";
  for (int i = 0; i < sizeof(tempDeviceAddress); i++) {
    sprintf(str, "%02X", tempDeviceAddress[i]); // Format as two uppercase hex digits
    strMAC.concat(':');
    strMAC.concat(str);
  }
  return strMAC;
}


// --- ESP-IDF WiFi Event Handler ---
// This handler uses ESP-IDF events, which are more direct and reliable
// for low-level WiFi state changes like STA_START.
static void EspWifiEventHandler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    debugln("EspWifiEventHandler - Event handler triggered."); // Added debug print
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        debugln("EspWifiEventHandler - WiFi Station Mode Started. Initializing ESP-NOW...");
#if DEBUG > 0
        serial_port.flush(); // Flush serial buffer
#endif
        debugln("EspWifiEventHandler - Adding small delay before ESP-NOW init...");
        delay(50); // Add a small delay (adjust if needed)
        debugln("EspWifiEventHandler - Delay finished. Initializing ESP-NOW...");

        // Init ESP-NOW - Moved here to ensure WiFi STA is started
        if (esp_now_init() != ESP_OK) { // Check return code for ESP32
          debugln("EspWifiEventHandler - Error initializing ESP-NOW");
#if DEBUG > 0
          serial_port.flush(); // Flush serial buffer
#endif
          // Consider adding error handling or retry logic here
          sensor_data_sent = true; // Allow deep sleep even on error
        } else {
          debugln("EspWifiEventHandler - ESP-NOW initialized");
          debug("EspWifiEventHandler - Time (ms): ");
          debugln(millis());
#if DEBUG > 0
          serial_port.flush(); // Flush serial buffer
#endif

          // Register the send callback function
          if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
              debugln("EspWifiEventHandler - Error registering ESP-NOW send callback");
#if DEBUG > 0
              serial_port.flush(); // Flush serial buffer
#endif
              // Consider error handling
              sensor_data_sent = true; // Allow deep sleep even on error
          } else {
             // Register peer - Moved here
            esp_now_peer_info_t peerInfo = {}; // Initialize struct to zeros
            // Alternatively: memset(&peerInfo, 0, sizeof(peerInfo));

            memcpy(peerInfo.peer_addr, broadcastAddress, 6);
            peerInfo.channel = ESPNOW_CHANNEL; // Set peer channel explicitly
            peerInfo.encrypt = false; // No encryption
            peerInfo.ifidx = WIFI_IF_STA; // Explicitly set interface to Station mode

            // Add peer - Moved here
            esp_err_t add_peer_result = esp_now_add_peer(&peerInfo);
            if (add_peer_result != ESP_OK) {
              debug("EspWifiEventHandler - Failed to add peer. Error: ");
              debugln(add_peer_result); // Print the specific error code
#if DEBUG > 0
              serial_port.flush(); // Flush serial buffer
#endif
              // Consider error handling
              sensor_data_sent = true; // Allow deep sleep even on error
            } else {
              debugln("EspWifiEventHandler - Peer added successfully");
#if DEBUG > 0
              serial_port.flush(); // Flush serial buffer
#endif

              // --- Prepare and Send ESP-NOW Message based on Wakeup Cause ---
              esp_err_t result_send = ESP_FAIL; // Initialize with failure

              debug("EspWifiEventHandler - Current global_wakeup_cause: ");
              switch (global_wakeup_cause) { // Use global_wakeup_cause here
                case ESP_SLEEP_WAKEUP_TIMER:
                  debugln("Timer");
                  break;
                case ESP_SLEEP_WAKEUP_EXT0:
                  debug("External signal using RTC_GPIO (EXT0) on GPIO ");
                  debugln(RAIN_GAUGE_PIN);
                  break;
                case ESP_SLEEP_WAKEUP_UNDEFINED:
                  debugln("Undefined (e.g., power on reset)");
                  break;
                default:
                  debug("Other (");
                  debug(global_wakeup_cause);
                  debugln(")");
                  break;
              }
#if DEBUG > 0
              serial_port.flush(); // Flush serial buffer
#endif

              if (global_wakeup_cause == ESP_SLEEP_WAKEUP_EXT0) {
                  // External wake-up (EXT0) from Rain Gauge Pin
                  debug("EspWifiEventHandler - EXT0 Wakeup Block on GPIO ");
                  debug(RAIN_GAUGE_PIN);
                  debugln(". Preparing rain data.");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  debug("EspWifiEventHandler - lastProcessedRainTipMillis: ");
                  debugln(lastProcessedRainTipMillis);
#if DEBUG > 0
                  serial_port.flush();
#endif

                  // Increment the saved rain gauge tips count upon successful EXT0 wake-up
                  savedRainGaugeTips++;
                  debug("EspWifiEventHandler - Incremented savedRainGaugeTips. New value: ");
                  debugln(savedRainGaugeTips);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Prepare rain event message into myData.a
                  char temp_str[20];
                  sprintf(temp_str, "%d", savedRainGaugeTips);
                  strcpy(myData.a, "ESP32|RainEvent|Tips|"); // Start with "ESP32" prefix
                  strcat(myData.a, temp_str);

                  debug("EspWifiEventHandler - Rain Event Message to send (payload): ");
                  debugln(myData.a);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  debugln("EspWifiEventHandler - Attempting to send rain gauge data via ESP-NOW...");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  // Send rain gauge data
                  result_send = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
                  debug("EspWifiEventHandler - esp_now_send status (rainData): ");
                  debugln(result_send);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Reset the saved tip count after sending
                  debugln("EspWifiEventHandler - Resetting saved rain gauge tips after sending.");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  savedRainGaugeTips = 0; // Reset the persistent count after sending
                  lastProcessedRainTipMillis = millis(); // Update last processed time
                  debug("EspWifiEventHandler - lastProcessedRainTipMillis updated to ");
                  debugln(lastProcessedRainTipMillis);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Wait for the rain gauge pin to go LOW to prevent immediate re-wakeup
                  debug("EspWifiEventHandler - Before physical debounce, RAIN_GAUGE_PIN state: ");
                  debugln(digitalRead(RAIN_GAUGE_PIN));
                  debugln("EspWifiEventHandler - Waiting for RAIN_GAUGE_PIN to go LOW to debounce...");
                  unsigned long physical_debounce_timeout = millis();
                  while (digitalRead(RAIN_GAUGE_PIN) == HIGH && (millis() - physical_debounce_timeout < 500)) { // Max 500ms wait
                      delay(10);
                      debug("."); // Indicate waiting
                  }
                  debugln(""); // Newline after dots
                  debug("EspWifiEventHandler - After physical debounce, RAIN_GAUGE_PIN state: ");
                  debugln(digitalRead(RAIN_GAUGE_PIN));
                  if (digitalRead(RAIN_GAUGE_PIN) == HIGH) {
                      debugln("EspWifiEventHandler - RAIN_GAUGE_PIN still HIGH after debounce wait. May re-trigger.");
                  } else {
                      debugln("EspWifiEventHandler - RAIN_GAUGE_PIN is LOW. Ready for deep sleep.");
                  }
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  delay(50); // Small delay to allow serial prints to flush before deep sleep


              } else if (global_wakeup_cause == ESP_SLEEP_WAKEUP_TIMER || global_wakeup_cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
                  // Timer wake-up or initial boot: Perform sensor reading and send data
                  debugln("EspWifiEventHandler - Timer/Undefined Wakeup Block. Reading sensors.");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  float temperatureC;
                  float temperatureF;
                  float batVolt;
                  long ellapsedTime = millis();
                  char result[8];

                  // Power up DS18B20 before reading
                  digitalWrite(TEMP_POWER, HIGH);
                  delay(100); // Give sensor time to power up
                  debugln("EspWifiEventHandler - DS18B20 powered ON for reading.");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif


                  // Re-initialize DS18B20 and get device count on every wake-up for sensor readings
                  sensors.begin();
                  devCount = sensors.getDeviceCount();
                  debug("EspWifiEventHandler - DS18B20 Sensors found (devCount): ");
                  debugln(devCount);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Set values to send - Start with BOARD identifier
                  strcpy(myData.a, "ESP32"); // Use "ESP32" as the board identifier

                  if (devCount > 0) { // Only attempt to read if sensors are found
                      sensors.setResolution(12); // Ensure resolution is set
                      sensors.setWaitForConversion(false); // Request non-blocking conversion
                      sensors.requestTemperatures(); // Send the command to get temperatures
                      debugln("EspWifiEventHandler - Requested temperatures. Waiting for conversion...");
                      delay(750); // Wait for 12-bit conversion (max 750ms)

                      for (int i = 0; i < devCount; i++) {
                        temperatureC = sensors.getTempCByIndex(i);
                        temperatureF = temperatureC * 1.8 + 32;
                        sensors.getAddress(tempDeviceAddress, i);
                        strMAC = mac2string(); // Get DS18B20 sensor address string
                        debug("EspWifiEventHandler - Sensor Address: ");
                        debugln(strMAC);
#if DEBUG > 0
                        serial_port.flush(); // Flush serial buffer
#endif

                        debug("EspWifiEventHandler - Raw TempC: ");
                        debugln(temperatureC);
#if DEBUG > 0
                        serial_port.flush(); // Flush serial buffer
#endif

                        // Check for valid temperature readings (85.00 and -127.00 are error codes)
                        if (temperatureC == 85.00 || temperatureC == -127.00) {
                            debugln("EspWifiEventHandler - DS18B20 returned error value (85.00 or -127.00). Reporting 'SensorError'.");
                            strcat(myData.a, "|");
                            strcat(myData.a, strMAC.c_str());
                            strcat(myData.a, "|TempC|SensorError"); // Report error
                            strcat(myData.a, "|TempF|SensorError"); // Report error
                        } else {
                            // Append sensor data to the message string
                            strcat(myData.a, "|");
                            strcat(myData.a, strMAC.c_str());
                            strcat(myData.a, "|TempC|");
                            dtostrf(temperatureC, 6, 2, result); // Convert float to string
                            strcat(myData.a, result);
                            strcat(myData.a, "|TempF|");
                            dtostrf(temperatureF, 6, 2, result);
                            strcat(myData.a, result);

                            debug("EspWifiEventHandler - DS18B20 Temp: ");
                            debug(temperatureC);
                            debug("*C\t and ");
                            debug(temperatureF);
                            debugln("*F.");
#if DEBUG > 0
                            serial_port.flush(); // Flush serial buffer
#endif
                        }
                      }
                  } else {
                      debugln("EspWifiEventHandler - No DS18B20 sensors found. Appending 'NoTempSensor'.");
#if DEBUG > 0
                      serial_port.flush(); // Flush serial buffer
#endif
                      strcat(myData.a, "|NoTempSensor|"); // Indicate no temp sensor
                  }

                  ellapsedTime = millis();
                  debug("EspWifiEventHandler - Temp collected Time (ms): ");
                  debugln(ellapsedTime);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Power off DS18B20 after reading
                  digitalWrite(TEMP_POWER, LOW);
                  digitalWrite(LED_PIN, LOW); // Turn off LED
                  debugln("EspWifiEventHandler - DS18B20 powered off.");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Read battery voltage
                  debugln("EspWifiEventHandler - Reading battery voltage...");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  digitalWrite(ADC_GND, LOW); // Set ADC_GND low for analog read
                  delay(50); // Small delay before analog read
                  int rawADC = analogRead(ANALOG_PIN);
                  digitalWrite(ADC_GND, HIGH); // Set ADC_GND high after analog read

                  // Assuming a voltage divider where the measured voltage is 1/5th of the actual battery voltage
                  // With ADC_11db attenuation, 4095 corresponds to approximately 3.3V
                  batVolt = (rawADC / 4095.0) * 3.3 * 5.0; // Adjusted calculation for voltage divider

                  debug("EspWifiEventHandler - Battery Voltage (ADC Raw): ");
                  debugln(rawADC); // Print raw ADC value for debugging
                  debug("EspWifiEventHandler - Battery Voltage (Calculated): ");
                  debugln(batVolt);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Append battery voltage data to the message string
                  strcat(myData.a, "|BatV|");
                  dtostrf(batVolt, 6, 2, result);
                  strcat(myData.a, result);
                  ellapsedTime = millis();
                  debug("EspWifiEventHandler - Battery Voltage collected Time (ms): ");
                  debugln(ellapsedTime);
                  debug("EspWifiEventHandler - Full Sensor Message to send: \"");
                  debug(myData.a);
                  debugln("\"");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif

                  // Send sensor data
                  debugln("EspWifiEventHandler - Attempting to send sensor data via ESP-NOW...");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  result_send = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
                  debug("EspWifiEventHandler - esp_now_send status (sensorData): ");
                  debugln(result_send);
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  delay(50); // Small delay to allow serial prints to flush before deep sleep

              } else {
                  // Other wake-up cause (shouldn't happen with current config)
                  debug("EspWifiEventHandler - Wakeup cause: Other (");
                  debug(global_wakeup_cause);
                  debugln("). No specific data to send.");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  delay(50); // Small delay to allow serial prints to flush before deep sleep
                  sensor_data_sent = true; // Indicate nothing to send, allow quick deep sleep
              }

              if (result_send == ESP_OK) {
                  debugln("EspWifiEventHandler - ESP-NOW send request successful");
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
              } else if (result_send != ESP_FAIL) { // Only print error if send was attempted and failed
                  debug("EspWifiEventHandler - Error sending ESP-NOW message: ");
                  debugln(result_send); // Print the specific error code
#if DEBUG > 0
                  serial_port.flush(); // Flush serial buffer
#endif
                  digitalWrite(LED_PIN, HIGH); // Indicate failure
                  sensor_data_sent = true; // Allow deep sleep even on error
              }
            }
          }
        }
    }
}


// --- Setup Function ---
void setup() {
  // Initialize Serial communication for debugging
  debugBegin(115200);
  debugln("\n--- Booting ESP32 ---"); // Added initial print
#if DEBUG > 0
  serial_port.flush(); // Flush serial buffer
#endif

  // Get the wake-up cause and store it globally
  global_wakeup_cause = esp_sleep_get_wakeup_cause();

  // Print wake-up cause
  debug("Setup - Wakeup cause: ");
  switch (global_wakeup_cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
      debugln("Timer");
      break;
    case ESP_SLEEP_WAKEUP_EXT0: // Changed back to EXT0
      debug("External signal using RTC_GPIO (EXT0) on GPIO ");
      debugln(RAIN_GAUGE_PIN); // Changed back to EXT0
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      debugln("Undefined (e.g., power on reset)");
      break;
    default:
      debug("Other (");
      debug(global_wakeup_cause);
      debugln(")");
      break;
  }
#if DEBUG > 0
  serial_port.flush(); // Flush serial buffer
#endif


  // Configure the OTA pin as an input with internal pull-up resistor.
  // This ensures the pin is HIGH by default if not connected to GND.
  pinMode(OTAPin, INPUT_PULLUP);

  // Give a short delay to allow the pin state to stabilize after boot
  delay(100);

  // Read the state of the OTA pin
  int pinState = digitalRead(OTAPin);

  // Get chip information
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  // Get flash chip size
  uint32_t flash_size = 0;
  esp_flash_t* flash = esp_flash_default_chip;
  if (esp_flash_get_size(flash, &flash_size) != ESP_OK) {
      flash_size = 0; // Indicate unknown size on error
    }


  // Check the pin state to decide whether to activate OTA or run Sensor Task/Rain Gauge Event and deep sleep
  if (pinState == LOW) {
    // --- Activate OTA Mode ---
    debugln("Setup - GPIO 23 is LOW. Activating OTA mode.");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    // Initialize LED pin and turn it on for OTA mode
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Turn LED on to indicate OTA mode

    // Print MCU specific information
    debugln("--- MCU Information ---");
    debug("Chip Model: ");
    debugln(chip_info.model == CHIP_ESP32 ? "ESP32" : "Unknown");
    debug("Chip Revision: ");
    debugln(chip_info.revision);
    debug("Number of Cores: ");
    debugln(chip_info.cores);
    debug("Flash Chip Size: ");
    debug(flash_size / (1024 * 1024));
    debugln("MB");
    debug("MAC Address: ");
    debugln(WiFi.macAddress());
    debugln("-----------------------");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    // Print WiFi credentials for debugging (REMOVE OR COMMENT OUT IN PRODUCTION)
    debug("Setup - Attempting to connect to SSID: ");
    debugln(ssid);
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif
    // WARNING: Printing password to serial is a security risk.
    // Only do this for debugging purposes and remove in production code.
    debug("Setup - Using Password: ");
    debugln(password);
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    // Connect to WiFi network using credentials from credentials.h
    debug("Setup - Connecting to WiFi...");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif
    WiFi.begin(ssid, password); // ssid and password are now defined in credentials.h
    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      debug(".");
#if DEBUG > 0
      serial_port.flush(); // Flush serial buffer
#endif
    }
    debugln("\nSetup - WiFi connected");
    debug("Setup - IP address: ");
    debugln(WiFi.localIP());
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    // Setup mDNS responder. Allows accessing the device using a hostname like esp32-ota.local
    if (!MDNS.begin("esp32-ota")) { // You can change "esp32-ota" to your desired hostname
      debugln("Setup - Error setting up MDNS responder!");
#if DEBUG > 0
      serial_port.flush(); // Flush serial buffer
#endif
    } else {
      debugln("Setup - mDNS responder started");
#if DEBUG > 0
      serial_port.flush(); // Flush serial buffer
#endif
    }

    // Start ElegantOTA server
    ElegantOTA.begin(&server); // Start ElegantOTA
    server.begin(); // Start the WebServer
    debugln("Setup - HTTP server started. ElegantOTA enabled.");
    debugln("Setup - Access OTA update page at:");
    debug("http://");
    debug(WiFi.localIP().toString());
    debugln("/update");
    debugln("or http://esp32-ota.local/update");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


  } else {
    // --- Run Sensor Task or Rain Gauge Event and Enter Deep Sleep ---
    debugln("Setup - GPIO 23 is HIGH. Running either Sensor Task or Rain Gauge Event.");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    // Print MCU specific information
    debugln("--- MCU Information ---");
    debug("Chip Model: ");
    debugln(chip_info.model == CHIP_ESP32 ? "ESP32" : "Unknown");
    debug("Chip Revision: ");
    debugln(chip_info.revision);
    debug("Number of Cores: ");
    debugln(chip_info.cores);
    debug("Flash Chip Size: ");
    debug(flash_size / (1024 * 1024));
    debugln("MB");
    debug("MAC Address: ");
    debugln(WiFi.macAddress());
    debugln("-----------------------");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    // Initialize pins for sensor task or rain gauge
    pinMode(LED_PIN, OUTPUT); // Initialize LED pin
    digitalWrite(LED_PIN, LOW); // Ensure LED is off initially

    // Configure the rain gauge pin as an input with PULL-DOWN resistor
    // to match the HIGH level wake-up configuration.
    pinMode(RAIN_GAUGE_PIN, INPUT_PULLDOWN);


    // Initialize sensor-related pins and ADC attenuation once after boot
    pinMode(TEMP_POWER, OUTPUT);
    digitalWrite(TEMP_POWER, LOW); // Ensure TEMP_POWER is LOW initially
    pinMode(ADC_GND, OUTPUT);
    pinMode(TEMP_DATA, INPUT); // Explicitly set TEMP_DATA pin as input
    analogSetAttenuation(ADC_11db); // Configure ADC attenuation for the analog pin

    debugln("Setup - completed for Sensor Task (initial pin setup).");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    // Set the board ID for the sensor data struct once
    // These are now handled within EspWifiEventHandler for each message type
    // strcpy(sensorData.board_id, "ESP32");
    // strcpy(rainData.board_id, "ESP32");


    // --- ESP-NOW Initialization and Sending (Triggered by WiFi Event) ---

    // Create default event loop (necessary for esp_event_handler_instance_register)
    esp_event_loop_create_default();

    // Register the ESP-IDF WiFi event handler
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &EspWifiEventHandler, NULL, NULL);

    // Set WiFi to Station mode
    WiFi.mode(WIFI_STA);

    // Explicitly set WiFi channel and start the driver for ESP-NOW
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE); // Set channel
    esp_wifi_start(); // Use esp_wifi_start()

    // The rest of the ESP-NOW initialization and sending
    // will happen inside the EspWifiEventHandler when WIFI_EVENT_STA_START occurs.

    // Wait for the data to be sent (or failure indicated) before deep sleep
    debugln("Setup - Waiting for data to be sent via ESP-NOW...");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif
    unsigned long espnow_timeout = millis();
     // Wait up to 10 seconds for data to be sent (adjust as needed)
    while (!sensor_data_sent && (millis() - espnow_timeout < 10000)) {
        delay(10);
    }

    if (!sensor_data_sent) {
        debugln("Setup - Timeout waiting for data to be sent. Entering deep sleep anyway.");
#if DEBUG > 0
        serial_port.flush(); // Flush serial buffer
#endif
    } else {
        // --- Cleanup ESP-NOW and WiFi before Deep Sleep ---
        debugln("Setup - Data sent successfully. Cleaning up ESP-NOW and WiFi...");
#if DEBUG > 0
        serial_port.flush(); // Flush serial buffer
#endif
        debugln("Setup - Adding small delay before ESP-NOW deinit...");
        delay(50); // Small delay before deinit
        esp_now_deinit();
        debugln("Setup - ESP-NOW deinitialized.");
        esp_wifi_stop();
        debugln("WiFi stopped.");
        // It might also be necessary to delete the event loop, but this can sometimes cause issues.
        // esp_event_loop_delete_default(); // Uncomment with caution if needed
#if DEBUG > 0
        serial_port.flush(); // Flush serial buffer
#endif
    }


    // --- GPIO Configuration for Deep Sleep Power Saving ---
    // Set all used non-RTC GPIOs to a low-power state.
    // It's crucial to ensure these pins are not floating or drawing current.
    // Explicitly list all GPIOs that you use or want to put into a known state.
    // Do NOT touch GPIOs 6-11 (connected to flash memory).
    // GPIO 1 and 3 are usually serial TX/RX, often left alone.
    // GPIO 23 (OTAPin) is checked at boot, then can be set.
    // GPIO 2 (LED_PIN) is already handled.
    // GPIO 16 (TEMP_POWER) is already LOW.
    // GPIO 17 (ADC_GND) is already HIGH.
    // GPIO 4 (TEMP_DATA) is an input.
    // GPIO 34 (ANALOG_PIN) is an input.
    // GPIO 14 (RAIN_GAUGE_PIN) is RTC GPIO for wakeup (input_pulldown).

    debugln("Configuring non-RTC GPIOs for deep sleep...");

    // Example of setting commonly available non-RTC GPIOs to INPUT_PULLDOWN
    // Adjust this list based on your specific board and usage!
    // If a pin is connected to something that pulls it HIGH, INPUT_PULLDOWN will draw current.
    // In such cases, OUTPUT LOW or INPUT_PULLUP (if it matches external hardware) might be needed.
    // Always consult your board's schematic.
    pinMode(0, INPUT_PULLDOWN);
    pinMode(5, INPUT_PULLDOWN);
    pinMode(12, INPUT_PULLDOWN);
    pinMode(13, INPUT_PULLDOWN);
    pinMode(15, INPUT_PULLDOWN);
    pinMode(25, INPUT_PULLDOWN);
    pinMode(26, INPUT_PULLDOWN);
    pinMode(27, INPUT_PULLDOWN);
    pinMode(32, INPUT_PULLDOWN);
    pinMode(33, INPUT_PULLDOWN);
    // Explicitly set TEMP_DATA to INPUT_PULLDOWN for deep sleep
    pinMode(TEMP_DATA, INPUT_PULLDOWN);
    // OTAPin (GPIO23)
    pinMode(OTAPin, INPUT_PULLDOWN); // After boot check, can be put into low power state

    // --- Configure Deep Sleep Wake-up Sources and Enter Deep Sleep ---

    // Print the state of the rain gauge pin right before deep sleep
    debug("Setup - RAIN_GAUGE_PIN (GPIO ");
    debug(RAIN_GAUGE_PIN);
    debug(") state before deep sleep: ");
    debugln(digitalRead(RAIN_GAUGE_PIN) == HIGH ? "HIGH" : "LOW");
#if DEBUG > 0
    serial_port.flush(); // Ensure this debug print is flushed
#endif


    // Configure timer wake-up for periodic readings
    esp_sleep_enable_timer_wakeup(periodicSleepTime);

    // Configure external wake-up from the rain gauge pin (EXT0)
    // The pin must be an RTC GPIO.
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM, level);
    // level is 0 for low, 1 for high. Use 1 for rising edge on pull-down pin.
    debug("Setup - Enabling EXT0 wakeup on GPIO ");
    debug(RAIN_GAUGE_PIN);
    debugln(" with HIGH level.");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif
    esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(RAIN_GAUGE_PIN), 1); // Wake on high level
    debugln("Setup - EXT0 wakeup configured.");
#if DEBUG > 0
    serial_port.flush(); // Flush serial buffer
#endif


    debug("Setup - Entering deep sleep for ");
    debug(periodicSleepTime);
    debug(" microseconds or until rain gauge pin (GPIO ");
    debug(RAIN_GAUGE_PIN);
    debugln(") goes HIGH...");
#if DEBUG > 0
    serial_port.flush(); // Final flush before sleep
#endif
    esp_deep_sleep_start();
  }
}

// --- Loop Function (only active in OTA mode) ---
void loop() {
  // Only run server.handleClient() and ElegantOTA.loop() if in OTA mode
  if (digitalRead(OTAPin) == LOW) {
    server.handleClient();
    ElegantOTA.loop();
  }
  // If not in OTA mode, loop() does nothing and the ESP32 will be in deep sleep.
}
