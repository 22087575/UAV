// Libraries
#include <Wire.h>
// Setupfuncties
void setupSensor1() {};
void setupSensor2() {};
void setupPi() {};
void setup()
{
 Wire.begin();
 Serial.begin(115200);
 setupSensor1();
 setupSensor2();
 setupPi();
}
// Alle functies
void leesSensor1() {};
void leesSensor2() {};
void leesPi() {};
void voerPracticum1Uit() {};
void voerPracticum2Uit() {};
void regelaar() {};
void aansturingMotoren() {};
//
const long SENSOR1CYCLUSTIJD = 10;
const long SENSOR2CYCLUSTIJD = 50;
const long REGELCYCLUSTIJD = 100;
const long PICYCLUSTIJD = 500;
long tijd;
long oudeSensor1tijd = 0;
long oudeSensor2tijd = 0;
long oudePiTijd = 0;
long oudeRegeltijd = 0;
int practicumNummer;
const int PRACT1 = 1;
const int PRACT2 = 2;
void loop()
{
 tijd = millis();
 // Lees alle sensoren inclusief de Pi
 if (tijd - oudeSensor1tijd > SENSOR1CYCLUSTIJD) // 10 ms
 {
 oudeSensor1tijd = tijd;
 leesSensor1();
 }
 if (tijd - oudeSensor2tijd > SENSOR2CYCLUSTIJD) // 50 ms
 {
 oudeSensor2tijd = tijd;
 leesSensor2();
 }
 if (tijd - oudePiTijd > PICYCLUSTIJD) // 500 ms
 {
 oudePiTijd = tijd;
 leesPi();
 }
 // Selecteer het practicum
 switch (practicumNummer)
 {
 case PRACT1:
 voerPracticum1Uit();
 break;
 case PRACT2:
 voerPracticum2Uit();
 break;
 default:
 //C Statements
 ;
 }
 // Stuur alle actuatoren aan
 if (tijd - oudeRegeltijd > REGELCYCLUSTIJD) // 100 ms
 {
 oudeRegeltijd = tijd;
 regelaar();
 aansturingMotoren();
 }
}
