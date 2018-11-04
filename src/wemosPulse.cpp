#include <wemosPulse.h>
void setup(){
  #define DEBUGMIO
  #ifdef DEBUGMIO
	mySerial.begin(9600);
  delay(10000);
	#endif
	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin();
	delay(100);
	DEBUG_PRINT("Booting!");


  initFirst();
  DEBUG_PRINT("initFirst DONE!");
  calculate_DC();
  DEBUG_PRINT("calculate_DC 1!");
  calculate_DC();
  DEBUG_PRINT("calculate_DC 2!");


}
void loop(){
  readSensor();
  //float value=slope(int *samples)
  delay(10);
}
