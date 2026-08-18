#include <WiFi.h> 

#include <WiFiUdp.h> 


const char* ssid = "moto g54 5G_2518"; 

const char* password = "ayush123"; 

const char* centerIP = "10.193.222.101"; // <-- Set Central ESP32 IP here 

const int udpPort = 8888; 

  

WiFiUDP udp; 

const int STATUS_LED = 2; // Onboard LED (Active HIGH) 

  

void setup() { 

  Serial.begin(115200); 

  pinMode(STATUS_LED, OUTPUT); 

  

  Serial.println("\n[INIT] Right Node ESP32 Starting..."); 

  WiFi.mode(WIFI_STA); 

  WiFi.begin(ssid, password); 

  

  while (WiFi.status() != WL_CONNECTED) { 

    delay(500); 

    Serial.print("."); 

  } 

  

  Serial.println("\n[CONNECTED] Right Node Ready!"); 

  Serial.print("Right Node IP: "); 

  Serial.println(WiFi.localIP()); 

} 

  

void loop() { 

  if (WiFi.status() == WL_CONNECTED) { 

    long rssiSum = 0; 

    for (int i = 0; i < 5; i++) { 

      rssiSum += WiFi.RSSI(); 

      delay(10); 

    } 

  

    int avgRssi = rssiSum / 5; 

    String payload = "RSSI,A101,R," + String(avgRssi); 

  

    udp.beginPacket(centerIP, udpPort); 

    udp.print(payload); 

    udp.endPacket(); 

  

    // Local Serial monitor log 

    Serial.printf("Sent: %-16s | Signal: %3d dBm\n", payload.c_str(), avgRssi); 

  

    digitalWrite(STATUS_LED, HIGH); 

    delay(40); 

    digitalWrite(STATUS_LED, LOW); 

  } else { 

    Serial.println("[WARNING] Connection lost. Reconnecting..."); 

    WiFi.begin(ssid, password); 

  } 

  delay(150); 

} 