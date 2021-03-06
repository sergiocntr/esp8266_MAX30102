#ifndef wifimio_h
#define wifimio_h
#include <ESP8266WiFi.h>
#include "wemosMaxConfig.h"
#include "debugutils.h"
uint8_t connLAN(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.config(ip, gateway, subnet); // Set static IP (2,7s) or 8.6s with DHCP  + 2s on battery
  for (int i=0; i <= 5; i++){
    if(WiFi.status() == WL_CONNECTED)
    {
      return 1;
    }
    delay(700);
  }
  WiFi.mode(WIFI_OFF); //energy saving mode if local WIFI isn't connected
  WiFi.forceSleepBegin();
  delay(10);
  return 0;
}
uint8_t connINTERNET(WiFiClient c){
  unsigned long timeout = millis();
  while (!c.connect(host, httpPort)) {
    if (millis() - timeout > 5000) {
      DEBUG_PRINT("conn internet FAIL! ");
      return  0;
    }
    ESP.wdtFeed();
    delay(500);
  }
  return 1;
}
#endif
