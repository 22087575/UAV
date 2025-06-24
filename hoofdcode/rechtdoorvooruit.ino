// libraries
  #include <Wire.h>
  #include <VL53L0X.h>
  #include <SPI.h>  //log
  #include <SD.h>   //log


// defines
  #define MPU_ADDR 0x68
  #define GYRO_SCALE 65.5  // 65.5 Voor ±500 °/s bereik 131.0 voor 250 bereik
  #define maxBufferSize 4096  // log


// globale variabelen
  long offsetZ = 0;
  const int samples = 250;

  unsigned long previousMillis = 0;
  const long interval = 2; // ms
  float angleZ = 0.0; // geïntegreerde hoek in graden

  int pwm;

float error_a = 0, error_oud_a = 0, d_error_a = 0, errorSom_a = 0; // afstandsregelaar
float error_b = 0, error_oud_b = 0, d_error_b = 0, errorSom_b = 0; // parallelregelaar
float error_theta = 0, error_oud_theta = 0, d_error_theta = 0, errorSom_theta = 0; // hoekregelaar

// Setpoints:
const float setpoint_a = 1000; // gewenste afstand tot doel (mm), pas aan!
const float setpoint_b = 200;  // gewenste afstand tot wand (mm), pas aan!
const float setpoint_theta = 0.0; // parallel aan wand

// PID parameters (voorbeeld, tunen voor jouw systeem)
float Kp_a = 0.005, Kd_a = 0.001, Ki_a = 0.0;
float Kp_b = 0.01,  Kd_b = 0.002, Ki_b = 0.0;
float Kp_theta = 0.03, Kd_theta = 0.01, Ki_theta = 0.0;

// instellen van pinnummers
  const int programmerPin1 = 0;
  const int programmerPin2 = 1;
  const int diepOntlading = 2;
  const int relaisFans = 3;
  const int relaisStepdownConverters = 4;
  const int motorzijkantPWM = 5;
  const int motorZijkantHoogLaag = 6;
  const int motorZijkantLaagHoog = 7;
  const int IN1 = 8;
  const int IN2 = 9;
  const int ENB = 10;
  const int IN3 = 11;
  const int IN4 = 12;   // Motor links IN4 (richting)
  const int ENA = 13;   // Motor links ENA (PWM)
  const int serieleCommunicatie1 = 18;
  const int serieleCommunicatie2 = 19;
  const int sda = 20;
  const int scl = 21;
  const int segment1 = 22;
  const int segment2 = 23;
  const int segment3 = 24;
  const int segment4 = 25;
  const int segment5 = 26;
  const int segment6 = 27;
  const int segment7 = 28;
  const int segment8 = 29;
  const int segment9 = 30;
  const int segment10 =31;
  const int XSHUT1 = 47;
  const int XSHUT2 = 48;
  const int XSHUT3 = 49;
  const int DO = 50;
  const int DI = 51;
  const int CLK = 52;
  const int CS = 53;
  const int stroomSensor = A0;

// regelaar instellingen:
  bool simulatie = false;
  const long cyclustijd = 10;      // ms;       Regelaar wordt 100x per seconde ververst.
  long t_oud, t_nw;                // ms, ms;
  float dt = 1;                    // s;        Nodig vanwege d_error / dt in de regelaar
  const float m = 1.42;             // kg;       Massa van de UAV
  const float Iz = 0.0296;          // kgm2;     Massatraagheid van de UAV
  float Fx, Fy = 0;          // N, N, Nm; Stuurkrachten en stuurmoment
  float Mz ;
  const float Mstoor = 0.01;       // Nm;       Stoormoment door helicoptereffect van de blowers
  const float Fmin = -0.1879, Fmax = 0.21; // Maximering van stuwkracht voor meer realistische simulatie
  float ax, ay, alfa;              // m/s2;     Versnellingen en hoekversnelling
  float vx    = 0.0, sx    = 0.0;  // m/s, m;   Beginwaarden tbv simulatie
  float vy    = 0.0, sy    = 0.0;  // m/s, m;   Beginwaarden tbv simulatie
  float omega = 0.0, theta = 0.0;  // m/s, m;   Beginwaarden tbv simulatie
  float Kp = 1.08, Kd = 1.08, Ki = 1;    // regelaarparameters PID-regelaar  Kp = 6.75, Kd = 2.7, Ki = 1
  float error, error_oud, d_error, errorSom; // De errors voor een PID-regelaar
  float errorPID, d_errorPID, errorSomPID, error_oudPID;
  float errorPD,  d_errorPD, errorSomPD, error_oudPD;
  float sp;            // Setpoint, nodig om sprongresponsie van 0 naar 1 te simuleren

  const long simulatietijd_in_s = long(10); // s; De simulatietijd is instelbaar
  const long simulatietijd = simulatietijd_in_s * long(1000); // ms
  const int pixels = 499;          // aantal weer te geven punten in de Serial plotter
  int pixel = 0;                   // teller van het aantal reeds weergegeven punten
  const float tijdPerPixel = simulatietijd / pixels;
  double tijdVoorNwePixelPlot;


// Tof functie voor simulaties
   float ToF() {
     float afstand;
     return (afstand);
  } // einde functie uitlezen tof voor simulatie

// functie voor aansturen stuwmotor links
  int pwm_links() {
    float freq;
    if (Mz >= 0) {
      digitalWrite(IN3, LOW);   // vooruit
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, constrain(map((int)(644.25 * Mz / 2 + 50.394), 0, 250, 0, 255), 0, 255));
    } else {
      digitalWrite(IN3, HIGH);    // achteruit
      digitalWrite(IN4, LOW);
      analogWrite(ENA, constrain(map((int)(-697.58 * Mz / 2 + 49.565), 0, 250, 0, 255), 0, 255));
    }
  } // einde functie motor links


// functie voor aansturen stuwmotor rechts
  int pwm_rechts() {
    float freq;
    if (Mz >= 0) {
      digitalWrite(IN1, LOW);   // vooruit  LOW
      digitalWrite(IN2, HIGH);  //HIGH
      analogWrite(ENB, constrain(map((int)(300 * Mz / 2 + 150), 0, 250, 0, 255), 0, 255));
    } else {
      digitalWrite(IN1, HIGH);    // achteruit
      digitalWrite(IN2, LOW);
      analogWrite(ENB, constrain(map((int)(-300 * Mz / 2 + 150), 0, 250, 0, 255), 0, 255));
    }
  } // einde functie motor rechts


// functie aansturen afmeer motor
  int pwm_midden() {
    float freq;
    if (Fx >= 0) {
      digitalWrite(motorZijkantHoogLaag, LOW);  // low
      digitalWrite(motorZijkantLaagHoog, HIGH); // high
      // Vooruit: freq = 1425.1 * Fy - 38.111
      freq = 1594.1 * Fx - 25.111;
    } else {
      digitalWrite(motorZijkantHoogLaag, HIGH); // high
      digitalWrite(motorZijkantLaagHoog, LOW);  // low
      // Achteruit: freq = -1594.7 * Fy + 44.753
      freq = -1594.7 * Fx + 25.753;
    }
    pwm = constrain(map((int)freq, 0, 255, 0, 255), 120, 170);
    analogWrite(motorzijkantPWM, pwm);
  } // einde functie aansturen afmeer motor


// functie om aan aansturing van motoren aan te roepen
  void motoraansturing() {
    digitalWrite(relaisFans, HIGH);
    digitalWrite(relaisStepdownConverters, HIGH);
    pwm_links();     // Bereken en schrijf alle uitgangen
    pwm_rechts();
    pwm_midden();
  } // einde voor aanroep functie van alle motoren


// functie voor plotten van simulatie
   void plot() {
    tijdVoorNwePixelPlot = tijdVoorNwePixelPlot + tijdPerPixel;
    Serial.print(Fx * 10);
    Serial.print(" ");
    Serial.print(ax * 10);
    Serial.print(" ");
    Serial.print(vx * 10);
    Serial.print(" ");
    Serial.println(sx * 10);
    pixel = pixel + 1;
  } // einde functie plotten simulatie


// functie voor poolplaatsing PD
  void PoolplaatsingPD() {
    const float Re = 0.3, Im = 0.6, pool3 = 0.5; // Poolplaatsing Im(pool) = 2 x Re(pool): Doorschot = 20%  0.3141592654  0.6283185307
    //  Berekening PD - parameters adhv de poolplaatsing
    Kp = (Re * Re + Im * Im) * m;
    Kd = 2 * Re * m;
  
  } // einde functie poolplaatsing

// functie voor poolplaatsing PID
  void PoolplaatsingPID() {
    const float Re = 0.1, Im = 0.2, pool3 = 1;
    Kp = (Re * Re + Im * Im + 2 * Re * pool3) * Iz;
    Kd = (2 * Re + pool3) * Iz;
    Ki = (Re * Re + Im * Im) * pool3 * Iz;
    
  } // einde functie poolplaatsing

void Poolplaatsingrechtvooruit() {
    const float Re = 0.1, Im = 0.2, pool3 = 1;
    Kp = (Re * Re + Im * Im + 2 * Re * pool3) * Iz;
    Kd = (2 * Re + pool3) * Iz;
    Ki = (Re * Re + Im * Im) * pool3 * Iz;
    
  } // einde functie poolplaatsing rechtvooruit
// PID-regelaar:
  void RegelaarPID() {
    PoolplaatsingPID();
    float sppid = 1;
    error_oudPID = errorPID;
    errorPID = sppid - theta;
  
//    Serial.print("kp kd ki (polen): ");
//    Serial.print(Kp);
//    Serial.print(Kd);
//    Serial.print(Ki);
    Serial.print(" Mz: ");
    Serial.println(Mz);
    d_errorPID = errorPID - error_oudPID;
    errorSomPID = errorSomPID + errorPID * dt;
    Mz = Kp * errorPID + Kd * d_errorPID / dt + Ki * errorSomPID;
    Mz = constrain(Mz, -0.7, 0.7); // Limiteer stuurmoment, pas aan voor jouw systeem
    alfa = (Mz + Mstoor) / Iz;     // bereken de hoekversnelling
  }


// functie PD regelaar
  void RegelaarPD() {
    PoolplaatsingPD();
    float sppd = 200;
    if(-10 < (sppd-sx) < 10){
      Fx=0;  
    }
      error_oudPD = errorPD;
      errorPD = sppd - sx;
      d_errorPD = errorPD - error_oudPD;
      errorSomPD = errorSomPD + errorPD * dt;
      Fx = (Kp * errorPD + Kd * d_errorPD / dt)/1000; // + Ki * errorSom;
      constrain(Fx, Fmin, Fmax);
      ax = Fx / m;     // bereken de versnelling
      
  } // einde functie PD-regelaar

  void Regelaarrechtvooruit() {
    Poolplaatsingrechtvooruit();
    float sppd = 700;
    if(-10 < (sppd-sx) < 10){
      Mz=0;  
    }
      error_oudPD = errorPD;
      errorPD = sppd - sx;
      d_errorPD = errorPD - error_oudPD;
      errorSomPD = errorSomPD + errorPD * dt;
      Mz = (Kp * errorPD + Kd * d_errorPD / dt)/1000; // + Ki * errorSom;
      constrain(Mz, -0.7, 0.7);
      ax = Mz / m;     // bereken de versnelling
      
  } // einde functie PD-regelaar
// functie voor aansturing langs de muur
void RegelaarLangsDeMuur(float afstandTotDoel, float afstandTotWand, float hoekTotWand) {
  int pwmLinks, pwmRechts;
  // Afstandsregelaar (vooruit/achteruit, richting a)
  error_oud_a = error_a;
  error_a = setpoint_a - afstandTotDoel;
  d_error_a = error_a - error_oud_a;
  errorSom_a += error_a * dt;
  float Fvooruit = Kp_a * error_a + Kd_a * d_error_a / dt + Ki_a * errorSom_a;

  // Parallelregelaar (afstand tot wand, richting b)
  error_oud_b = error_b;
  error_b = setpoint_b - afstandTotWand;
  d_error_b = error_b - error_oud_b;
  errorSom_b += error_b * dt;
  float Fwand = Kp_b * error_b + Kd_b * d_error_b / dt + Ki_b * errorSom_b;

  // Hoekregelaar (parallel aan wand)
  error_oud_theta = error_theta;
  error_theta = setpoint_theta - hoekTotWand;
  d_error_theta = error_theta - error_oud_theta;
  errorSom_theta += error_theta * dt;
  // Versterk de hoekcorrectie met een factor, bijvoorbeeld 2.0
  float Fhoek = (Kp_theta * error_theta + Kd_theta * d_error_theta / dt + Ki_theta * errorSom_theta) * 2.0;

  // Dode zone toepassen
  if (abs(Fvooruit) < 0.05) Fvooruit = 0;
  if (abs(Fhoek)    < 0.05) Fhoek    = 0;

  // Combineer krachten voor links/rechts motoren:
  // - Fvooruit: beide motoren gelijk
  // - Fhoek: links +, rechts -
  float F_links  = Fvooruit - Fhoek;
  float F_rechts = Fvooruit + Fhoek;

  // Dode zone voor gecombineerde krachten
  if (abs(F_links)  < 0.05) F_links  = 0;
  if (abs(F_rechts) < 0.05) F_rechts = 0;

  // Middenmotor krijgt alleen Fwand (voor afstand tot wand)
  Fx = Fwand;

  // Zet PWM voor links/rechts motoren
  int maxPwm = 127;
  int minPwm = 30; // minimale PWM om motor te laten draaien

  pwmLinks = constrain((int)(abs(F_links) * 50), 0, 255);
  if (pwmLinks > 0 && pwmLinks < minPwm) pwmLinks = minPwm;

  pwmRechts = constrain((int)(abs(F_rechts) * 50), 0, 255);
  if (pwmRechts > 0 && pwmRechts < minPwm) pwmRechts = minPwm;

  // Richting en aansturing
  if (F_links > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, pwmLinks);
  } else if (F_links < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, pwmLinks);
  } else {
    analogWrite(ENA, 0); // Stop motor
  }

  if (F_rechts > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENB, pwmRechts);
  } else if (F_rechts < 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENB, pwmRechts);
  } else {
    analogWrite(ENB, 0); // Stop motor
  }

  pwm_midden(); // gebruikt Fx

  // Print logregel in CSV-formaat: tijd, hoek, Fvooruit, Fwand, Fhoek, afstand
  Serial.print(millis()); Serial.print(",");
  Serial.print(hoekTotWand); Serial.print(",");
  Serial.print(Fvooruit); Serial.print(",");
  Serial.print(Fwand); Serial.print(",");
  Serial.print(Fhoek); Serial.print(",");
  Serial.println(afstandTotDoel);
  Serial.println(pwmRechts);
  Serial.println(pwmLinks);
}

// data voor loggen
struct SensorData {
  long time;
  float value1;
  float value2;
};  // einde struct voor loggen data

// structure voor gelezen data ToF sensoren
struct ToFdata{
   float afstand1;
   float afstand2;
   float afstand3;
   float hoek;
   float dx;
 };  // einde structure voor ToF data


// Aanmaken van objecten
  VL53L0X ToF1;  // maak een object van VL53L0X en noem deze ToF1
  VL53L0X ToF2;  // maak een object van VL53L0X en noem deze ToF2
  VL53L0X ToF3;  // maak een object van VL53L0X en noem deze ToF3

  ToFdata data;

      static const unsigned numElements = maxBufferSize / sizeof(SensorData); // Bereken hoeveel elementen (structs) er weggeschreven kunnen worden in de buffer.
      static SensorData buffer[numElements];
      static boolean stopWriting = false;
      static const long samplingTimeDataLogging = 1000;  // ms, bepaalt frequentie waarmee sensordata wordt weggeschreven.
      static const long timeStopWriting = 80000;  // ms, bepaalt frequentie waarmee sensordata wordt weggeschreven.
      static long t_lastDataLog = 0, t_new;  // ms, ms;
      static int currentIndex = 0;
      unsigned fileCounter = 0;
      String filename;
    
// functie voor data schrijven voor log
void writeDataToFile(byte *buffer, unsigned bufferSize) {
  
  do {
    filename = "data" + String(fileCounter) + ".bin";
    fileCounter++;
  } while (SD.exists(filename)); // Controleer of het bestand al bestaat
  File dataFile = SD.open(filename, FILE_WRITE);
  
  if (dataFile) {
    Serial.println("Schrijven naar " + filename + "...");
    // Schrijven van de buffer naar het bestand
    dataFile.write(buffer, bufferSize);
    
    // Sluiten van het bestand
    dataFile.close();
    Serial.println("Schrijven naar " + filename + " voltooid");
  } else {
    Serial.println("Fout bij het openen van het bestand");
  }
} // einde functie data schrijven voor log


// functie voor log data oproepen
void logSensorData() {
  unsigned long t_new = millis();

  // start loggen als we nog niet gestopt zijn
  if (!stopWriting && t_new - t_lastDataLog >= samplingTimeDataLogging) {
    t_lastDataLog = t_new;

    buffer[currentIndex].time = t_new;
    buffer[currentIndex].value1 = data.dx;
    buffer[currentIndex].value2 = pwm=0;
    
    currentIndex++;

    if (currentIndex >= numElements) {
      // buffer is vol, schrijven naar SD
      writeDataToFile((byte*)buffer, sizeof(buffer));
      currentIndex = 0;
      stopWriting = false;  // evt resetten om door te loggen, of true zetten om te stoppen
    }
  }

  // stoppen met loggen na een bepaalde tijd
  if (t_new > timeStopWriting) {
    if (!stopWriting) {
      // schrijf rest buffer naar SD voordat stoppen
      writeDataToFile((byte*)buffer, currentIndex * sizeof(SensorData));
      stopWriting = true;
      Serial.println("Loggen gestopt");
    }
  }
} // einde functie voor log data oproepen

// setup voor ToF-sensoren
void setupToF(){
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);

  // Zet alle sensors uit
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(100);

  // Zet ToF1 aan en initialiseer
  digitalWrite(XSHUT1, HIGH);
  delay(100);
  if (ToF1.init()) {
    Serial.println("ToF1 init OK");
    ToF1.setAddress(0x30);
    ToF1.setMeasurementTimingBudget(200000);
  } else {
    Serial.println("ToF1 init Mislukt!");
  }

  // Zet ToF2 aan en initialiseer
  digitalWrite(XSHUT2, HIGH);
  delay(100);
  if (ToF2.init()) {
    Serial.println("ToF2 init OK");
    ToF2.setAddress(0x31);
    ToF2.setMeasurementTimingBudget(200000);
  } else {
    Serial.println("ToF2 init Mislukt!");
  }

  // Zet ToF3 aan en initialiseer
  digitalWrite(XSHUT3, HIGH);
  delay(100);
  if (ToF3.init()) {
    Serial.println("ToF3 init OK");
    ToF3.setAddress(0x32);
    ToF3.setMeasurementTimingBudget(200000);
  } else {
    Serial.println("ToF3 init Mislukt!");
  }

} // einde setup ToF


// setup
void setup() {

  Serial.begin(57600); // stel de baudrate in op 9600
  Wire.begin(); // schakel de I2C aan
  setupToF(); // roep de ToF-setup aan
  
// OUTPUT:
  analogWrite(motorzijkantPWM, 0);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(relaisFans, OUTPUT);
  pinMode(relaisStepdownConverters, OUTPUT);

  pinMode(motorzijkantPWM, OUTPUT);
  pinMode(motorZijkantHoogLaag, OUTPUT);
  pinMode(motorZijkantLaagHoog, OUTPUT);

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Power management
  Wire.write(0x00);
  Wire.endTransmission(true);
    
  // Stel gyroscoop bereik in op ±500 °/s (0x08 in register 0x1B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // Gyro config register
  Wire.write(0x08);  // oxo8 voor ±500 °/s 0x00 voor 250 graden
  Wire.endTransmission(true);
  calculateOffsets();  // Bepaal gyroscoopoffset

// simulatie setup PD
  if (simulatie) {
    Serial.print("F a v x");  // De legenda
    if (tijdPerPixel < cyclustijd) Serial.println("___XXXXXX____tijdPerPixel.<.cyclustijd____XXXXXX");
    else Serial.println();
  }
  t_oud = millis();
  if (simulatie) tijdVoorNwePixelPlot = t_oud;
  if (not simulatie) sx = ToF();
  error = sp - sx;
  PoolplaatsingPD();
  if (simulatie) plot();


// zonder simulatie setup
  if(!simulatie){
    // Initialisatie van de SD-kaart
    if (!SD.begin(CS)) {
      Serial.println("\nInitialisatie van de SD-kaart mislukt!");
      return;
      }
      Serial.println("\nInitialisatie van de SD-kaart voltooid.");
    }

} // einde main setup


// functie om ToF sensoren uit te lezen
void UitlezenToF(){
    
  const int ConstanteAfstand = 351.2;  // Constante afstand tussen twee ToF-sensoren (235)
  const int CorrectieToF1 = 10.7; // negatief 14.5
  const int CorrectieToF2 = 3.2; // negatief 25.6
  const int CorrectieToF3 = 16.5; // positief 16.5
  
  data.afstand1 = ToF1.readRangeSingleMillimeters() - CorrectieToF1; // lees ToF1 uit met als eenheid mm
  data.afstand2 = ToF2.readRangeSingleMillimeters() - CorrectieToF2; // lees ToF2 uit met als eenheid mm
  data.afstand3 = ToF3.readRangeSingleMillimeters() - CorrectieToF3; // lees ToF3 uit met als eenheid mm

  // print de afstanden
  // Serial.print("Sensor 1: ");
  Serial.print(data.afstand1);
  Serial.print(" ");
  // Serial.print(" mm\t");

  // Serial.print("Sensor 2: ");
  Serial.print(data.afstand2);
  Serial.print(" ");
  // Serial.print(" mm\t");

//  // Serial.print("Sensor 3: ");
  Serial.print(data.afstand3);
  Serial.print(" ");
//  // Serial.println(" mm");

  data.hoek = BerekenHoek(data.afstand1, data.afstand2, ConstanteAfstand); // hoek is de teruggeven waarde van BerekenHoek functie
  // print de hoek tussen ToF1 en ToF2
  // Serial.print("Hoek: ");
  Serial.println(data.hoek);

  data.dx = BerekenAfstandAfmeren(data.afstand1, data.afstand2);
} // einde uitlezen ToF


// Functie om hoek te berekenen met ToF sensoren
float BerekenHoek(float D1, float D2, float d){
  float verschil = D2 - D1; // bereken het verschil tussen afstand ToF1 tot de wand en afstand ToF2 tot de wand
  float Theta = (verschil / d); // bereken thetá
  Theta = (180 * Theta) / PI;  // omzetten naar graden
  return Theta; // geef thetá terug
} // einde berekenen hoek ToF


// Functie om afstand voor afmeren te berekenen
float BerekenAfstandAfmeren(float x1, float x2){
  float dx = (x1 + x2) / 2;
  return dx;
} // einde functie BerekenAfstandAfmeren


// berekenen van offsets van gyroscoop
void calculateOffsets() {

  Serial.println("Berekenen van offsets...");
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Startadres gyroscoopdata
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    Wire.read(); Wire.read(); // negeer X
    Wire.read(); Wire.read(); // negeer Y
    offsetZ += (Wire.read() << 8 | Wire.read()); // alleen Z
    delay(5);
  }
  offsetZ /= samples;
  Serial.print("Offset Z: "); Serial.println(offsetZ);
} // einde berekenen offests gyro


// uitlezen van gyroscoop
void lezenGyro(){
    unsigned long currentMillis = micros();
    if (currentMillis - previousMillis >= interval) {
      float dt = (currentMillis - previousMillis) / 1000000.0;
      previousMillis = currentMillis;

      // Lees gyro Z-as
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, true);

      Wire.read(); Wire.read(); // negeer X
      Wire.read(); Wire.read(); // negeer Y
      int16_t gyroZ = (Wire.read() << 8 | Wire.read()) - offsetZ;

      float gyroZ_dps = gyroZ / GYRO_SCALE;
      angleZ += gyroZ_dps * dt;
      theta = -angleZ;

//      Serial.print("Angle Z (deg): ");
//      Serial.println(angleZ);
    }
} // einde lezen gyro


// void loop, blijft herhaald worden
void loop() {
  t_nw = millis();    // Lees de tijd
  if (t_nw - t_oud > cyclustijd) {
    dt = (t_nw - t_oud) * 0.001;
    t_oud = t_nw;

// simulatie
    if (simulatie) {
      // SIMULATIEMODE
      sx = sx + vx * dt; // Bereken x
      vx = vx + ax * dt; // Bereken v
      RegelaarPD();

      if (pixel == pixels) while (true);       // Stop met plotten als buffer vol is
      if (t_nw > tijdVoorNwePixelPlot) plot(); // Plot enkel op juiste momenten
    } 
  }

// zonder simulatie
  if (!simulatie) {
    // Alleen bij niet-simulatie: States afhandelen via seriële input

    static int startStates = 0;
    static bool Geprint = false;

    // Verwerking seriële commando's
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input == "Idle"            ){ startStates =  0; Geprint = false; }
      if (input == "Afmeren"         ){ startStates =  1; Geprint = false; }
      if (input == "LangsDeMuurVaren"){ startStates =  2; Geprint = false; }
      if (input == "STOPPEN"         ){ startStates =  3; Geprint = false; }
      if (input == "Keren"           ){ startStates =  4; Geprint = false; }
      if (input == "ArucoVinden"     ){ startStates =  5; Geprint = false; }
      if (input == "Stoppen"         ){ startStates =  6; Geprint = false; }
      if (input == "stop"            ){ startStates = 10; Geprint = false; }
      if (input == "ToF"             ){ startStates = 11; Geprint = false; }
      if (input == "gyro"            ){ startStates = 12; Geprint = false; }
      if (input == "beide"           ){ startStates = 13; Geprint = false; }
      if (input == "test"            ){ startStates = 14; Geprint = false; }
    }

    // Switch voor states alleen als we NIET simuleren
    switch (startStates) {
      case 0: // Idle
        if (!Geprint) { Serial.println("State: Idle"); Geprint = true; }
        digitalWrite(relaisFans, LOW);
        digitalWrite(relaisStepdownConverters, LOW);
        calculateOffsets();  // Bepaal gyroscoopoffset
        break;
      case 1: // Afmeren
        if (!Geprint) { Serial.println("State: Afmeren"); Geprint = true; }
        UitlezenToF();
        lezenGyro();
        Serial.print("Fx: ");
        Serial.print(Fx);
        Serial.print(" ");
        Serial.print("PWM: ");
        Serial.println(pwm);
        Serial.print("Theta: ");
        Serial.print(theta);
        sx=data.dx;
        RegelaarPD();
        RegelaarPID();
        motoraansturing();
        logSensorData();
        break;
      case 2: // LangsDeMuurVaren
        if (!Geprint) { Serial.println("State: LangsDeMuurVaren"); Geprint = true; }
        digitalWrite(relaisFans, HIGH);                  // <-- relais aan
        digitalWrite(relaisStepdownConverters, HIGH);    // <-- relais aan
        UitlezenToF();
        lezenGyro();
        // Gebruik ToF en hoek voor regeling:
        // data.dx = afstand tot doel (vooruit/achteruit)
        // data.afstand1 of data.afstand2 = afstand tot wand (kies sensor die naar wand wijst)
        // data.hoek = hoek t.o.v. wand
        RegelaarLangsDeMuur(data.afstand3, data.dx, data.hoek);
        break;
      case 3: // STOPPEN voor de muur
        if (!Geprint) { Serial.println("State: STOPPEN"); Geprint = true; }
        break;
      case 4: // Keren (90 graden draaien)
        if (!Geprint) { Serial.println("State: keren"); Geprint = true; }
        break; 
      case 5: // ArUco vinden
        if (!Geprint) { Serial.println("State: ArucoVinden"); Geprint = true; }
        delay(2000);
        Serial.println("yes"); // Signal Raspberry Pi to start

        if (Serial.available()) {
          String data = Serial.readStringUntil('\n');
          data.trim();
          if (data == "no_marker") {
            Serial.println("No marker received.");
          } 
          else if (data == "done") {
            Serial.println("Session complete.");
          } 
          else {
            Serial.print("Received: ");
            Serial.println(data);  // Data
            }
          }      
        break;
      case 6: // Stoppen
        if (!Geprint) { Serial.println("State: Stoppen"); Geprint = true; }
        break;        
      case 10: // stop alles
        if (!Geprint) { Serial.println("Alle sensoren uit."); Geprint = true; }
        break;
      case 11: // Alleen ToF
        if (!Geprint) { Serial.println("ToF actief"); Geprint = true; }
        UitlezenToF();
        break;
      case 12: // Alleen gyroscoop
        if (!Geprint) { Serial.println("Gyroscoop actief"); Geprint = true; }
        lezenGyro();
        RegelaarPID();
        digitalWrite(relaisFans, HIGH);
        digitalWrite(relaisStepdownConverters, HIGH);
        pwm_links();     // Bereken en schrijf alle uitgangen
        pwm_rechts();
        Serial.print("Theta: ");
        Serial.print(theta);
        break;
      case 13: // Beide actief
        if (!Geprint) { Serial.println("ToF + Gyro actief"); Geprint = true; }
        UitlezenToF();
        lezenGyro();
        break;
      case 14: // Teststate
        if (!Geprint) { Serial.println("Teststate actief"); Geprint = true; }
        UitlezenToF();
        sx = data.dx;
        RegelaarPD();
        motoraansturing();
        break;
    }
  }
}
