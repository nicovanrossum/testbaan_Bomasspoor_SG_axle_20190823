
#include <TimerOne.h>
#include <avr/pgmspace.h>

/*
 testbaan pointBank tweede fase

Gaat ervoor zorgen dat de pointstraten gedurende een xx tijd gevoed worden (als blok 4 en blok 5) door het blok waar de trein naar toe wordt gestuurd
als het sein groen toont is de rijweg gereed en zal het eigen blok maar ook het destination blok voeding krijgen. De trein begint 
te rijden in eigen blok en zou geen spanning meer ontvangen op de pointBank. Het point wijst naar het blok dat in de rijweg 
ligt en wordt gebruikt om te kunnen kijken welke blokvoeding gebruikt moet worden voor de pointBank.

  The circuit:
  - on only two points the endswitch will be used. 4 lines will be available for the OC32 and ROCrail sensorring.
  4 Analog input signals are used to gain the three positions of the froglegs; Rightguiding, Leftguiding and on the move.
  On the move is detected by a low voltage (0 Volt) comming from the resistor network
  In de tussenstand, de tijd dat het point van stand A naar stand B loopt is het ingangssignaal hoog
  beide schakelaars zitten in een pointcontact. Door gebruik te maken van een weerstandsbank met 5 4K7 weerstanden wordt het
  mogelijk om 3 verschillende analoge spanningen aan te bieden op de analoge ingang. De 5 Volt is de tussen stand. 
  Stand schakelaar 1 levert ongeveer 2,5 Volt vanwege de spanningsdeling. Schakelcontact 2 brengt twee parallele 4K7 weerstanden naar 
  ground. Op die manier wordt de spanning rond de 1,5 Volt.

Axcel detectors for detecting train in the pointblocks

Two digital inputs will be used to detect the availability of a train in  a pointblock in the time frame 
By using the interupt option the number of axcels are counted from max to zero. This availability is given to 2 signals for the dimamo OC32.

Usage of the LEDs
  De LED kunnen worden ingezet om de status aan te geven
  LED links is voor sturing van pointBank 1
  LED rechts is voor sturing van pointBank 2 
  Both LEDs on means test / debugging modeis enabled

  For debugging purposes it is posible to mount a jumper on A5 to ground. 
  The program will give status information on the serial connection with the PC.
  It will ask the user from which area information need to be seen.
  The areas are: 
    1) pointposition = pp, 
    2) greenLEDs = gl
    3) measured values pointsleg = mvp, 
    4) measured values greenleds = mvg, 
    5) matrix of measurements = mom,
    6) enabiling relais positions =erp.

  created by Nico van Rossum feb 2019
  modified 2 feb 2019
  By Nico

  This example code is in the Bomasspoor domain.

*/
      // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
      // scope assistent

String testArea, destr, source, dt, resp, failArea;
// the next strings will be transeffer to the EEPROM during compilation time
// this bytes will be freed for the variables.
const char string_0[] PROGMEM = "the test and debug jumper is mounted";
const char string_1[] PROGMEM = "you will be asked which area you want to test";
const char string_2[] PROGMEM = "You can choose between:";
const char string_3[] PROGMEM = "pointposition = a";
const char string_4[] PROGMEM = "greenLEDs = b";
const char string_5[] PROGMEM = "measured values froglegs= c";
const char string_6[] PROGMEM = "measured values greenleds = d";
const char string_7[] PROGMEM = "matrix of measurements = e";
const char string_8[] PROGMEM = "enabling relaispositions = f";
const char string_9[] PROGMEM = "relais sequential test = g";
const char string_10[] PROGMEM = "What's your choise?";
// than the setup table to refer tot the strings
const char* const diagnoseStrings[] PROGMEM ={string_0, string_1, string_2, string_3, string_4, string_5, string_6,
      string_7,string_8, string_9, string_10};
const char gs_0[] PROGMEM = "greenDetect0 combi O0+/O1+ (blok 0d en blok 1d) sv4= ";
const char gs_1[] PROGMEM = "greenDetect1 combi O0-/O1- (blok 0a en blok 1a) sv5= "; 
const char gs_2[] PROGMEM = "greenDetect2 combi O2+/O3+ (blok 2d en blok 3d) sv6= ";   
const char gs_3[] PROGMEM = "greenDetect3 combi O2-/O3- (blok 2a en blok 3a) sv7= "; 
const char ws1_0[] PROGMEM = "pointBank1 point_0 positionSwitch";
const char ws1_1[] PROGMEM = "pointBank1 point_1 positionSwitch";
const char ws2_2[] PROGMEM = "pointBank2 point_2 positionSwitch";
const char ws2_3[] PROGMEM = "pointBank2 point_3 positionSwitch";
const char* const rapportStrings[] PROGMEM ={gs_0, gs_1, gs_2, gs_3, ws1_0, ws1_1, ws2_2, ws2_3};
const char q1[] PROGMEM = "do you want to see the next loop j or n";
const char* const questionStrings[] PROGMEM ={q1};
const char plws1_0[] PROGMEM = "pointFrog in pointBank1 point_0 in positie:";
const char plws1_1[] PROGMEM = "pointFrog in pointBank1 point_1 in positie:";
const char plws2_2[] PROGMEM = "pointFrog in pointBank2 point_2 in positie:";
const char plws2_3[] PROGMEM = "pointFrog in pointBank2 point_3 in positie:";
const char* const froglegStrings[] PROGMEM ={plws1_0, plws1_1, plws2_2, plws2_3};
const char trvlpath_0[] PROGMEM = "TP0_from B2a or B3a to B0d";
const char trvlpath_1[] PROGMEM = "TP1_from B2a or B3a to B1d";
const char trvlpath_2[] PROGMEM = "TP2_from B0d or B1d to B3a";
const char trvlpath_3[] PROGMEM = "TP3_from B0d or B1d to B2a";
const char trvlpath_4[] PROGMEM = "TP4_from B2d or B3d to B0a";
const char trvlpath_5[] PROGMEM = "TP5_from B2d or B3d to B1a";
const char trvlpath_6[] PROGMEM = "TP6_from B0a or B1a to B2d";
const char trvlpath_7[] PROGMEM = "TP7_from B0a or B1a to B3d";
const char trvlpath_8[] PROGMEM = "no train for green led";
const char* const trvlpathStrings[] PROGMEM ={trvlpath_0, trvlpath_1, trvlpath_2, trvlpath_3, trvlpath_4, trvlpath_5, trvlpath_6, trvlpath_7, trvlpath_8};
char printBuffer[80] ; // this buffer is used to print the above strings
int receivedChar;
int flushChar;
boolean newData = false;
boolean resp1Ok = false;
char endMarker = '\r';
const int selectTijd = 500; // Bepaal hiermee de tijd in milliseconden waarbinnen de vooorwaarde stabiel zijn.
// Time to drive through the pointblockarea, time that the right voltage, 
// for the destination block, is put on the tracks of the pointblockarea
const int timedriveThrough = 5000; // est.5000 (times the interrupt routine) is 5 second, 5000 milliseconds
int driveThrough0 = 100; // counter in interrupt routine two functions:
int driveThrough1 = 100; // counter in interrupt routine two functions:
bool  wheelDetect0 = LOW; // when no wheel is in between the IR-LED and the IR sensor
bool  wheelDetect1 = LOW; // the sensor will get light and will go LOW. This is the rest situation.
// Detection of a wheel results in an 'high' signal level
// first is during startup to calculate the correct averages (init is zeros)
// second is for no measurmwnts during the time that the train is travelling through the pointsection
bool travelPathready = LOW; // when HIGH the the loop tavelpath will only done once
// time to display error information
const int faultdisplay = 15000;
// keepalivecounter to toggle leds to show the system is operating
const int keepalivecounter =100; // half cycletime of keep alive beat
// number of times the measurements are done and the values are smoohed
const int countmeasurements = 10;
// cntm >> counts down the number of greenled measuremente within the interrupt routine
int cntm = 2;
// logic statement that the interupt routine has enough samples ready for calulating
bool statintcount = HIGH;
// time between measurements 50 milliseconden will be waited
// const int timebetweenmeasures = 5000;
// autodiasgnose mode, after one of the errors has been fired this mode will automaticaly show the problem on the LEDs
bool autodiagnose = LOW; // no autodiagnose
// 0 is no testmode ; comes from inputsignal on testpin. ( switch in open position)
// (switch pulls resistor to ground when enabled))
bool testmode = HIGH; // high means no testmode
bool testPin = HIGH; // initial state of testPin variable
// preparation for debug / test mode
int origin = 77; // shows where the failure comesfrom >> 77 is no error
int errorvalue = 0; // measured value in érror'situation for reporting
char resp1; // inputvariable to support the switch structures 
const int inforeadtime = 500; // time (in milliseconds) to wait for reading the information frn the PC screen
// code information for status leds
// in normal mode the leds are alternating in a low frequency approx 0,5 Hz
// in error mode there will be several speed modes approx 2 Hz, 5 Hz and 10 HZ
bool led1h = LOW;
bool led1l = HIGH;
bool ledphase = LOW;
// firering the timer for the interupt routine
const int timeroneinit = 1000; // 1000 micro seconds between the interupts , 1 ms
// ledtimerh and ledtimerl are manipulated in the nmerrorhandling routine to change the blinking frequncies of the leds
int ledtimerh = 100; // 100 times timeroneinit >> LEDs in high state >> results in 5Hz blinking
int ledtimerl = 100; // 100 times timeroneinit >> LEDs in low state >> results in 5Hz blinking
int blhi = 20; // this is the countdown variable for the high phase of the blinking
int bllo = 20; // this is the countdown variable for the low phase of the blinking
// newAvgMeasurement is used to tell the routine that there are new valid measurement values available
bool newAvgMeasurement = LOW;
// constants for detecting leg position in points due to analoge mesurements on analog ports A0, A1, A2, A3
const int underlow = 100; // bandwidth level low left guiding position
const int underhigh = 500; // bandwidth level high left guiding position
const int upperlow = 700; // bandwidth level low right guiding position
const int upperhigh = 850; // bandwidth level high right guiding position
const int underLEDlow = 600; // bandwidth level low greenLED
const int upperLEDhigh = 750; // bandwidth level high greenLED
int frogleg_0 = 0; // 2 = left guiding position, 1 = right guiding position, 0 = moving between positions, 3 = failure reading
int frogleg_1 = 0; // 2 = left guiding position, 1 = right guiding position, 0 = moving between positions, 3 = failure reading
int frogleg_2 = 0; // 2 = left guiding position, 1 = right guiding position, 0 = moving between positions, 3 = failure reading
int frogleg_3 = 0; // 2 = left guiding position, 1 = right guiding position, 0 = moving between positions, 3 = failure reading
int legpos = 0; // supports printing errorlist
bool groenstat23a = LOW; // HIGH = green signalling light from 2a or 3a to drive into point frogleg_0 going to blocks B0 or B1
bool groenstat01a = LOW; // HIGH = green signalling light from 0a of 1a to drive into point frogleg_2 going to blocks B2 or B3
bool groenstat23d = LOW; // HIGH = green signalling light from 2d of 3d to drive into point frogleg_3 going to blocks B0 or B1
bool groenstat01d = LOW; // HIGH = green signalling light from 0d of 1d to drive into point frogleg_1 going to blocks B2 or B3
bool pointBank0 = LOW; // pointbank0 not active
bool pointBank1 = LOW; // pointbank1 not active
byte detectPinpointbank0 = 2;      // select the pin d2 for the IR detection of pointbank0
byte detectPinpointbank1 = 3;      // select the pin d3 for the IR detection of pointbank1
byte led4Pin = 4;      // select the pin d4 for the infoLED4, also the stay alive LED
// option: byte enaPowerpin = 4;     // select pin d4 to enable power to the point districts
byte diagPin = 13;      // select the pin d13 for enabling the test modus
// the traincurrent for the two point districs are dirived from the destination block.
// because these voltages for both legs can be in both directions switching is done by a relais bank
// this relaisbank is controlled by the greensignalling leds and the legposition of the points.
int relaisPin[] = {5, 6, 7, 8, 9, 10, 11, 12};
    // select the pin d5 for relais 0
    // select the pin d6 for relais 1
    // select the pin d7 for relais 2
    // select the pin d8 for relais 3
    // select the pin d9 for relais 4
    // select the pin d10 for relais 5
    // select the pin d11 for relais 6
    // select the pin d12 for relais 7
// 
// bool relaisPinvalue[] = {HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW}; 
bool relaisPinvalue[] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};    
// bool relaisPinvalue[] = {HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW};
// bool relaisPinvalue[0] = LOW; // low will generate an active relais 0
// bool relaisPinvalue[1] = LOW; // low will generate an active relais 1
// bool relaisPinvalue[2] = LOW; // low will generate an active relais 2
// bool relaisPinvalue[3] = LOW; // low will generate an active relais 3
// bool relaisPinvalue[4] = LOW; // low will generate an active relais 4
// bool relaisPinvalue[5] = LOW; // low will generate an active relais 5
// bool relaisPinvalue[6] = LOW; // low will generate an active relais 6
// bool relaisPinvalue[7] = LOW; // low will generate an active relais 7
// preparation of measurement lists
// by using the 8 analog inputs of the NANO we can get quite some more information from our environment
// eg the quality degradation of the point position switches 
// an other item is we get information by one value on only one NANO pin of the position of the frogleg 
// a disadvantage is the expected variations of the measured values so averaging bij the NANO is essential.
int sensorPin[]= {A0, A1, A2, A3, A7, A4, A5, A6}; // list for the sensorpins
  // select the input pin for pointBank1 point_0 positionSwitch
  // select the input pin for pointBank1 point_1 positionSwitch
  // select the input pin for pointBank2 point_2 positionSwitch
  // select the input pin for pointBank2 point_3 positionSwitch
  // select the A7 input pin for greenDetect0 combi O0+/O1+ (blok 0d en blok 1d)
  // select the A4 input pin for greenDetect1 combi O0-/O1- (blok 0a en blok 1a)
  // select the A5 input pin for greenDetect2 combi O2+/O3+ (blok 2d en blok 3d)
  // select the A6 input pin for greenDetect3 combi O2-/O3- (blok 2a en blok 3a)
int sensorValue[8][countmeasurements+2]; // definitition of measurement matrix
// last eight (2-11)fields are to be use for the datagathering, field 1 is used for the sum and field 0 is for the calculated average
int sensorDefault = 0;

void setup() 
{
  char testArea[10] = "init";  // Input value from user which information must been shown during testing
  char failArea[15] = "failarea"; // tekst variable for the failing area
  destr = String("destination track");
  source = String("bron");
  dt = String("dest"); //destinationtrack
  resp = String("niets"); // setup resp(onse) variabele bij selectie van tetstoptions
  // = String(""); // input for check end of testperiod  
// write default values into output pins
   for (int m = 0; m<8; m++) {
       digitalWrite(relaisPin[m], relaisPinvalue[m]);  
       }
// this is the setup of the interupt routine. This routine ensures the analog reading of the green signalling leds
//
// Setup timerOne
//  
  Timer1.initialize(timeroneinit); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  // filling of the measurement matrix with a start default value (matrix 8 * 10)
  cleanMMM (); // clean MeasurMent Matrix
  // SETUP pinmodes
  // declare the IR axelcounter detector to the detetPinpointbank*
  pinMode(detectPinpointbank0, INPUT_PULLUP);
  pinMode(detectPinpointbank1, INPUT_PULLUP);
  // declare the led4Pin as an OUTPUT:
  pinMode(led4Pin, OUTPUT);
  // declaration of the testpin , facilitates the error diagnose mode with the osciloscope
  pinMode(diagPin, INPUT_PULLUP);
  // declare the relaispins to an output for driving the relay module
  for (int i=0; i < 8 ; i++){
    pinMode(relaisPin[i], OUTPUT);
    }
// initialize the USB connection to the PC for debugging time
  Serial.begin(9600); 
  while(!Serial);
   Serial.println("Connection OK");
   //eepromPrint(rapportStrings[1]);
// keepalive leds
// this is the setup of the interupt routine. This routine ensures the analog reading of the green signalling leds
  // setup hartbeat blinking >>  alternating leds every 2 seconds
  ledtimerh = keepalivecounter;
  led1h = LOW;
  ledtimerl = keepalivecounter;
  led1l = HIGH;               
}

void loop() {
    //digitalWrite( led4Pin, digitalRead(led4Pin)^1);
    testPin = digitalRead(diagPin); 
    if (testPin == LOW)
       {
        Serial.println("loop testPin is low");
        diagnosemode();
        }
    else {
    }
    if (testPin == LOW){  testmode = HIGH;} 
    if (testmode == HIGH || autodiagnose == HIGH ) 
       {resp =  1;} 
     // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
     // testing is done when no train is travelling through the pointblok;
     //digitalWrite( led4Pin, digitalRead(led4Pin)^1);
    averageReadings();
    // showavgReadings();
    greenSignaltest();
    //outGreendetect();
    //cleanMMM ();  // clean the measurement matrix
    // Serial.print(driveThrough0);
    // Serial.println("dr0");
    // Serial.println(pointBank0);
    if (driveThrough0 == 0 && pointBank0 == HIGH){
       detectFrogpositionPB0();
       Serial.println("Pointbank0 = high");
       setupTravelpath0();          // the travel path0 and the required relays can be setup
       // write relais patern
       digitalWrite(relaisPin[0], relaisPinvalue[0]);
       digitalWrite(relaisPin[2], relaisPinvalue[2]);
       digitalWrite(relaisPin[4], relaisPinvalue[4]);
       digitalWrite(relaisPin[6], relaisPinvalue[6]);
       Serial.println("Pointbank0 relais output done");
       driveThrough0 = timedriveThrough; 
       Serial.println("dT0"); 
       }
    // Serial.print(driveThrough1);
    // Serial.println("dr1");
    // Serial.println(pointBank1);
    if (driveThrough1 == 0 && pointBank1 == HIGH){
       detectFrogpositionPB1();
       Serial.println("Pointbank1 = high");
       setupTravelpath1();          // the travel path1 and the required relays can be setup
       digitalWrite(relaisPin[1], relaisPinvalue[1]);
       digitalWrite(relaisPin[3], relaisPinvalue[3]);
       digitalWrite(relaisPin[5], relaisPinvalue[5]);
       digitalWrite(relaisPin[7], relaisPinvalue[7]);
       Serial.println("Pointbank0 relais output done");
       driveThrough1 = timedriveThrough; 
       Serial.println("dT1"); 
       }   

    // this routine checks the availability for a green signalling led in pointbank0
    if (pointBank0 == HIGH)
          {
          if (driveThrough0 > 0) {
          // a train is moving true pointblock0, check for wheel
             // Serial.print("T0");
             // Serial.println(driveThrough0);
             wheelDetect0 = digitalRead(detectPinpointbank0);
             if (wheelDetect0 == HIGH) { 
              Serial.println("WD0H");         
              // when a wheel is detect the pointbank0 will get extra time so the train can get out of the point block
              driveThrough0 = timedriveThrough;
              }
         } 
          // turn all pointbank0 relais in position to enable the right travel path
          // due to negate logic of the relais driver circuitry, relais are 'off' by sending a HIGH from the arduino to the relaisXpin
      
          //digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          // Serial.println(" Pointbank0 there is a green LED signal");
          if (driveThrough0 <= 2){
              // reset the relais position again, less current for the 7805 5 Voltregulator
              digitalWrite(relaisPin[0], HIGH);
              digitalWrite(relaisPin[2], HIGH);
              digitalWrite(relaisPin[4], HIGH);
              digitalWrite(relaisPin[6], HIGH);
              Serial.println("reset relais dT0"); 
              }  
        
          // the train can drive through pointbank0
         }
    
    if (pointBank1 == HIGH)
          {
          // turn relais in fixed position to enable the right travel path
          // due to negate logic of the relais driver circuitry, relais are 'off' by sending a HIGH from the arduino to the relaisXpin
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          // Serial.println(" Pointbank1 there is a green LED signal");
          // there is a green LED signal
          //
          if (driveThrough1 > 0) {
             //Serial.print("T1");
             //Serial.println(driveThrough1);
             wheelDetect1 = digitalRead(detectPinpointbank1);
             // a train is moving true pointblock0, check for wheel
             if (wheelDetect1 == HIGH) {
                Serial.println("WD1H");
                // when a wheel is detect the pointbank1 will get extra time so the train can get out of the pointblock
                driveThrough1 = timedriveThrough;
                }
          }
          if (driveThrough1 <= 2 ){
              // reset the relais position again
              // less current for the 7805 5 Voltregulator
              digitalWrite(relaisPin[1], HIGH);
              digitalWrite(relaisPin[3], HIGH);
              digitalWrite(relaisPin[5], HIGH);
              digitalWrite(relaisPin[7], HIGH);
              Serial.println("reset relais dT1");               
             } 
          // the train can drive through pointblock1
          } 
     
  }

void cleanMMM (){
  // clean the measurement matrix
     for(int i = 0; i < 8; i++)  // these are 8 rows 
       {
        for(int j = 0; j < (countmeasurements+2); j++)  // these are 'countmeasures' coloms
            {
            sensorValue[i][j] = sensorDefault; // in every celcoordinate is 0 assigned
            }
        }   
    }

void eepromPrint(const char* eePrint) {
      strcpy_P(printBuffer,(char*)pgm_read_word(&(eePrint)));
      Serial.println (printBuffer);
      }

void recvOneChar(){
  nextChar:
  if (Serial.available() > 0) {
     receivedChar = Serial.read();
     // Serial.print("char 1 is ... ");
     // Serial.print(receivedChar,HEX);
     // Serial.write(int(receivedChar));
     }
  if (receivedChar == '\n'){
     newData =false;
     goto nextChar;
     }
  else {
    newData = true;
    }
}

void showNewData() {
    resp1 = receivedChar;
    newData = false; 
    resp1Ok = true;
}
/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
void timerIsr()
 {  
    // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
    // support blinking of status leds
    // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
    // led 1 and led2 give 3 states, by using the time domain 
    // (changing the blinking frequency more information can be seen by the two LEDs.
    if (ledphase == HIGH) {
       if (blhi <= ledtimerh) 
          {
          //digitalWrite( led4Pin, HIGH);
          blhi = blhi -1;
          }
       if (blhi == 0) 
          {
          bllo = ledtimerl;
          ledphase = LOW;
          }
       }
    if (ledphase == LOW) 
       {
       if (bllo <= ledtimerl) 
          {
          //digitalWrite( led4Pin, LOW);
          bllo = bllo-1;
          }
        if (bllo == 0) 
          {
           blhi = ledtimerh;
          ledphase = HIGH;
          }
       }
    // reading of all analog inputs for the number of countmeasurements  
    if (statintcount == LOW )
      {
       cntm = cntm -1;  // counter for making 'measurmentcount' measurments  
       // read the values from the sensors and store them each in their own list
       for (int i = 0; i < 8; i++){
          sensorValue[i][cntm] = analogRead(sensorPin[i]); 
          }
       if (cntm == 2)
          { 
          // max number of measurments is reached (countmeasurements is set in averageReadings() )
          // the main routine can start counting the smooth averages for this measurement batch
          statintcount = HIGH;
          }
      }
    // calculate the time a train can move throught the pointblok
    if (driveThrough0 > 0) {
          driveThrough0 = driveThrough0 -1 ;
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
         }
    if (driveThrough0 == 1) {
          pointBank0 = LOW; // pointbank should be empty now
         }
    if (driveThrough1 > 0) {
          driveThrough1 = driveThrough1 -1 ;
         }
    if (driveThrough1 == 1) {
            pointBank1 = LOW; // pointbank should be empty now
          }
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          // a train is moving true pointblock1
}

void averageReadings() {
// digitalWrite( led4Pin, digitalRead(led4Pin)^1); // line to use scope to measure output
  if (statintcount == HIGH ) {
      // measure and average values & store averaged values in lists
      // read the values from the leg point position sensors
     for (int n = 0; n < 8; n++) { 
         sensorValue[n][1] = 0; // empty the sum variable per sensor
     }
     for (int m = 0; m < 8; m++) {
        for (int i = 2; i < countmeasurements+2; i++) {
            sensorValue[m][1] = sensorValue[m][1] + (sensorValue[m][i]); // adding all measurement value from one sensorpin together 
            }
        sensorValue[m][0] = sensorValue[m][1]/countmeasurements; // calculating smoothed average
      }
    newAvgMeasurement = HIGH; // the determination of the input signals can start in the greenSignaltest() 
    cntm = countmeasurements + 2; // interrupt routine can start measuring again
    // in the matrix position 1 is used for calculating the sum of the readings
    // position 0 is used for the average value for that pin reading
    statintcount = LOW; // the interupt routine can start measuring again
   }
}

void greenSignaltest() {
  // greenDetect test
  // pointBank0:
  // groenstat23a comes from signal B2a and signal B3a
  // groenstat01d comes from signal B0d and signal B1d
  // pointBank1:
  // groenstat01a comes from signal B0a and signal B1a
  // groenstat23d comes from signal B2d and signal B3d
  // Both greenled signals before a pointarea dictate the permission for a train to enter the pointarea
  // Normally the green LED is 'off'. When a train gets permission to go beyond the signal the green LED goes on.
  // This is controlled by the Dinamo OC32 system which is controlled by the ROCrail software.
  // This ROCrail software is a safety system. It makes sure that only one train gets permission to enter the point Area
  // The two entry Greenled, which are high active are 'ORed' with two diodes. 
  // The reasult if one of the two Green leds get high the 'OR'ed result gets high too.
  // This is traditional diode OR-ing
  // This combined signal is entered at the 'groenstatXs' analog entries. A4 to A7.
  // This HIGH is a voltage higher than 4 volt 
  // It is measured by the b
  // AD convertor of the arduino with a aproximate value of 800.
  // In the program / sketch 600 is the limit for upperhigh.   
  // If the read out is more than 600 the signal is asumed to be HIGH.
  // If detemine an real LOW the signal value must be below 'underlow'.
  // A value inbetween is suspiscious and invokes an error message.
  // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
  if (driveThrough0 == 0 && newAvgMeasurement == HIGH) {
      // test groenstat23a
      // groenstat23a is actief
      //Serial.println(sensorValue[4][0]);
      if (sensorValue[4][0] > upperLEDhigh) {
        //digitalWrite( led4Pin, digitalRead(led4Pin)^1);
        groenstat23a = HIGH;
        pointBank0 = HIGH;
      }
       // groenstat23a is pasief
      else if (sensorValue[4][0] < underLEDlow) {
        // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
        groenstat23a = LOW;
      }
      // groenstat23a heeft problemen met de schakeling
      else if (sensorValue[4][0] > underLEDlow && sensorValue[4][0] < upperLEDhigh){
        origin = 4 ; //"groenstat23a";
        errorvalue = sensorValue[4][0];
        nmerrorhandling();
      }
      //digitalWrite( led4Pin, groenstat23a);
      // test groenstat01d
      // groenstat01d is actief
      //Serial.println(sensorValue[5][0]);
      if (sensorValue[5][0] > upperLEDhigh) {
        groenstat01d = HIGH;
        pointBank0 = HIGH;
      }
      // groenstat01d is pasief
      else if (sensorValue[5][0] < underLEDlow) {
        groenstat01d = LOW;
      }
      // groenstat01d heeft problemen met de schakeling
      else if (sensorValue[5][0] > underLEDlow && sensorValue[5][0] < upperLEDhigh){
        origin = 5 ; // "groenstat01d";
        errorvalue = sensorValue[5][0];
        nmerrorhandling();
      }
      //digitalWrite( led4Pin, groenstat01d);
      // test groenstat01a
      // groenstat01a is actief
      // Serial.println(sensorValue[6][0]);
      if (sensorValue[6][0] > upperLEDhigh) {
        groenstat01a = HIGH;
        pointBank1 = HIGH;
      }
      // groenstat01a is pasief
      else if (sensorValue[6][0] < underLEDlow) {
        groenstat01a = LOW;
      }
      // groenstat01a heeft problemen met de schakeling
      else if (sensorValue[6][0] > underLEDlow && sensorValue[6][0] < upperLEDhigh){
        origin = 6 ; // "groenstat01a";
        errorvalue = sensorValue[6][0];
        nmerrorhandling();
      }
      // digitalWrite( led4Pin, groenstat01a);
      // test groenstat23d
      // groenstat23d is actief
      // Serial.println(sensorValue[7][0]);
      if (sensorValue[7][0] > upperLEDhigh) {
        groenstat23d = HIGH;
        pointBank1 = HIGH;
        }
      // groenstat23d is pasief
      else if (sensorValue[7][0] < underLEDlow) {
        groenstat23d = LOW;
        }
      // groenstat23d heeft problemen met de schakeling
      else if (sensorValue[7][0] > underLEDlow && sensorValue[7][0] < upperLEDhigh){
        origin = 7 ; // "groenstat23d"
        errorvalue = sensorValue[7][0];
        nmerrorhandling();
        }
      // digitalWrite( led4Pin, groenstat23d);
  }
  newAvgMeasurement = LOW;
 
  }
void detectFrogposition(){
     detectFrogpositionPB0();
     detectFrogpositionPB1();
  }


void detectFrogpositionPB0(){
  // test position froglegs 
  // frogleg_0 and frogleg_1 are part of pointBank0
  // frogleg_2 en frogleg_3 are part of pointBank1
  // ........
  // frogleg_0 froglegs test
  // frogleg_0 froglegs are left guiding
  // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
  // Serial.println(sensorValue[0][0]);
   if (sensorValue[0][0] > upperhigh) {
      frogleg_0 = 2;
      }
    // frogleg_0 froglegs are right guiding
   else if (sensorValue[0][0] < upperlow && sensorValue[0][0] > underhigh) {
      frogleg_0 = 1;
      }
   // frogleg_0 froglegs are moving to other position
   else if (sensorValue[0][0] < underlow) {
      frogleg_0 = 0;
      origin = 8 ; // "frogleg_0";
      errorvalue = sensorValue[0][0];
      plmoveHandling();
      }
   // frogleg_0 heeft problemen met de schakeling
   else if (sensorValue[0][0] < underlow && sensorValue[0][0]< underhigh ) {
      frogleg_0 = 3;
      origin = 0 ; // "frogleg_0";
      errorvalue = sensorValue[0][0];
      nmerrorhandling();
      }
   else if (sensorValue[0][0] < upperlow && sensorValue[0][0]< upperhigh ) {
      frogleg_0 = 3;
      origin = 0 ; // "frogleg_0";
      errorvalue = sensorValue[0][0];
      nmerrorhandling();
      }
   else {
        ;
      }
   // frogleg_2 froglegs test
   // frogleg_2 froglegs are left guiding
   // Serial.println(sensorValue[2][0]);
   if (sensorValue[2][0] > upperhigh) {
      frogleg_2 = 2;
      }
    // frogleg_2 froglegs are right guiding
   else if (sensorValue[2][0] < upperlow && sensorValue[2][0] > underhigh) {
      frogleg_2 = 1;
      }
    // frogleg_2 froglegs are moving to other position
   else if (sensorValue[2][0] < underlow) {
      frogleg_2 = 0;
      origin = 10 ; // "frogleg_2";
      errorvalue = sensorValue[2][0];
      plmoveHandling();
      }
   // frogleg_2 heeft problemen met de schakeling
   else if (sensorValue[2][0] < underlow && sensorValue[2][0]< underhigh ) {
      frogleg_2 = 3;
      origin = 2 ; // "wsl"2;
      errorvalue = sensorValue[2][0];
      nmerrorhandling();
      }
   else if (sensorValue[2][0] < upperlow && sensorValue[2][0]< upperhigh ) {
      frogleg_2 = 3;
      origin = 2 ; // "frogleg_2";
      errorvalue = sensorValue[2][0];
      nmerrorhandling();
      }
   else {
       ;
       }
 travelPathready = HIGH;
 }

void detectFrogpositionPB1(){
  // frogleg_1 froglegs test
  // frogleg_1 froglegs are left guiding
  if (sensorValue[1][0] > upperhigh) {
      frogleg_1 = 2;
     }
   // frogleg_1 froglegs are right guiding
  else if (sensorValue[1][0] < upperlow && sensorValue[1][0] > underhigh) {
     frogleg_1 = 1;
     }
   // frogleg_1 froglegs are moving to other position
  else if (sensorValue[1][0] < underlow) {
     frogleg_1 = 0;
     origin = 9 ; // "frogleg_2";
     errorvalue = sensorValue[2][0];
     plmoveHandling();
     }
  // frogleg_1 heeft problemen met de schakeling
  else if (sensorValue[1][0] < underlow && sensorValue[1][0]< underhigh ) {
     frogleg_1 = 3;
     origin = 1 ; // "frogleg_1";
     errorvalue = sensorValue[1][0];
     nmerrorhandling();
     }
  else if (sensorValue[1][0] < upperlow && sensorValue[1][0]< upperhigh ) {
     frogleg_1 = 3;
     origin = 1 ; // "frogleg_1";
     errorvalue = sensorValue[0][0];
     nmerrorhandling();
     }
  else {
       ;
       }
  // frogleg_3 froglegs test
  // frogleg_3 froglegs are left guiding
  // Serial.println(sensorValue[3][0]);
   if (sensorValue[3][0] > upperhigh) {
     frogleg_3 = 2;
     }
   // frogleg_3 froglegs are right guiding
   else if (sensorValue[3][0] < upperlow && sensorValue[3][0] > underhigh) {
     frogleg_3 = 1;
     }
   // frogleg_3 froglegs are moving to other position
   else if (sensorValue[3][0] < underlow) {
      frogleg_3 = 0;
      origin = 11 ; // "frogleg_3";
      errorvalue = sensorValue[3][0];
      plmoveHandling();
     }
  // frogleg_3 heeft problemen met de schakeling
   else if (sensorValue[3][0] < underlow && sensorValue[3][0]< underhigh ) {
      frogleg_3 = 3;
      origin = 3 ; // "frogleg_3";
      errorvalue = sensorValue[3][0];
      nmerrorhandling();
     }
   else if (sensorValue[3][0] < upperlow && sensorValue[3][0]< upperhigh ) {
      frogleg_3 = 3;
      origin = 3 ; // "frogleg_3";
      errorvalue = sensorValue[3][0];
      nmerrorhandling();
      }
  else {
       ;
       }
 travelPathready = HIGH;
}

void plmoveHandling(){
  // will show itself at startup when buffer capacitors are loading
  Serial.print(origin);
  Serial.print("frogleg is moving");
  Serial.println(errorvalue);
  // delay(100); // wait for ending movenment
}
void setupTravelpath0() {
  // setting up relais position to enable voltages on the point tracks
  // Pointbank0
  // groenstat23a comes from signal B2a en signal B3a
  // groenstat01d comes from signal B0d en signal B1d
  // Pointbank1
  // groenstat01a comes from signal B0a en signal B1a
  // groenstat23d comes from signal B2d en signal B3d
  //
if (travelPathready == HIGH) {
  // Pointblockarea 0
  // train is going to travel from B2a or B3a to B0d
  // prerequisites: groenstat23a = 1 en frogleg_1 is right leading frogleg_1 = 1
  // Active relais are described, NOTE the relais module is driven LOW active 
  // to get a relais up a low level signal is send from de arduino ports
  // actions trackleg A: relais 0  = high, relais 4 = low
  // actions trackleg B: relais 2 = low, relais 6 = low
  //digitalWrite( led4Pin, digitalRead(led4Pin)^1);
 if  (groenstat23a == HIGH && frogleg_1 == 1){
      relaisPinvalue[0] = LOW;
      relaisPinvalue[2] = HIGH;
      relaisPinvalue[4] = HIGH;
      relaisPinvalue[6] = HIGH;
   // train is going to travel from B2a or B3a to B0d
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[0])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B2a or B3a to B0d");
      }
   // train is going to travel from B2a or B3a to B1d
   // prerequisites: groenstat23a = 1 en frogleg_1 is left guiding frogleg_1 = 2
   // Active relais are described, NOTE the relais module is driven LOW active 
   // to get a relais up a low level signal is send from de arduino ports
   // actions trackleg A: relais 0 = high, relais 4 = high
   // actions trackleg B: relais 2 = low, relais 6 = high
 else if (groenstat23a == HIGH && frogleg_1 == 2){
      relaisPinvalue[0] = LOW;
      relaisPinvalue[2] = HIGH;
      relaisPinvalue[4] = LOW;
      relaisPinvalue[6] = LOW;
   // train is going to travel from B2a or B3a to B1d
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[1])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B2a or B3a to B1d");
      }
 // train is going to travel from B0d or B1d to B3a
 // prerequisites: groenstat01d = 1 en frogleg_0 is right guiding frogleg_0 = 1
 // Active relais are described, NOTE the relais module is driven LOW active 
 // to get a relais up a low level signal is send from de arduino ports
 // actions trackleg A: relais 0 = low, relais 4 = low
 // actions trackleg B: relais 2 = high, relais 6 = low
 else if (groenstat01d == HIGH && frogleg_0 == 1){
      relaisPinvalue[0] = HIGH;
      relaisPinvalue[2] = LOW;
      relaisPinvalue[4] = HIGH;
      relaisPinvalue[6] = HIGH;
   // train is going to travel from B0d or B1d to B3a
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[2])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B0d or B1d to B3a");
      }
 // train is going to travel from B0d or B1d to B2a
 // prerequisites: groenstat01d = 1 en frogleg_0 is left guiding frogleg_0 = 2
 // Active relais are described, NOTE the relais module is driven LOW active 
 // to get a relais up a low level signal is send from de arduino ports
 // actions trackleg A: relais 0 = low, relais 4 = high
 // actions trackleg B: relais 2 = high, relais 6 = high
 else if (groenstat01d == HIGH && frogleg_0 == 2){
      relaisPinvalue[0] = HIGH;
      relaisPinvalue[2] = LOW;
      relaisPinvalue[4] = LOW;
      relaisPinvalue[6] = LOW;
    //train is going to travel from B0d or B1d to B2a
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[3])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B0d or B1d to B2a");
      } 
 else {
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[8])));
      Serial.println (printBuffer);
   // Serial.println ("no train for green led");
      }
   }
 groenstat23a = LOW; // HIGH = green signalling light from 2a or 3a to drive into point frogleg_0 going to blocks B0 or B1
 groenstat01d = LOW; // HIGH = green signalling light from 0d of 1d to drive into point frogleg_1 going to blocks B2 or B3
 travelPathready = LOW;
}
void setupTravelpath1() {
  // Pointbank1
  // groenstat01a comes from signal B0a en signal B1a
  // groenstat23d comes from signal B2d en signal B3d
  //
if (travelPathready == HIGH) {

 // pointBank1
 // train is going to travel from B2d or B3d to B0a
 // prerequisites: groenstat23d = HIGH en frogleg_2 is left guiding frogleg_2 = 1
 // Active relais are described, NOTE the relais module is driven LOW active 
 // to get a relais up a low level signal is send from de arduino ports
 // actions trackleg A: relais 1 = high, relais 5 = low
 // actions trackleg B: relais 3 = low, relais 7 = low
    if (groenstat23d == HIGH && frogleg_2 == 1){
      relaisPinvalue[1] = LOW;
      relaisPinvalue[3] = HIGH;
      relaisPinvalue[5] = HIGH;
      relaisPinvalue[7] = HIGH;
      // train is going to travel from B2d or B3d to B0a
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[4])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B2d or B3d to B0a");
      }
 // train is going to travel from B2d or B3d to B1a
 // prerequisites: groenstat23d = HIGH en frogleg_2 is right guiding frogleg_2 = 2
 // Active relais are described, NOTE the relais module is driven LOW active 
 // to get a relais up a low level signal is send from de arduino ports
 // actions trackleg A: relais 1 = high, relais 5 = high
 // actions trackleg B: relais 3 = low, relais 7= high
 else if (groenstat23d == HIGH && frogleg_2 == 2){
      relaisPinvalue[1] = LOW;
      relaisPinvalue[3] = HIGH;
      relaisPinvalue[5] = LOW;
      relaisPinvalue[7] = LOW;
   // train is going to travel from B2d or B3d to B1a
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[5])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B2d or B3d to B1a");
      }
 // train is going to travel from B0a or B1a to B2d
 // prerequisites: groenstat01a = HIGH en frogleg_3 is left guiding frogleg_3 = 1
 // Active relais are described, NOTE the relais module is driven LOW active 
 // to get a relais up a low level signal is send from de arduino ports
 // actions trackleg A: relais 1 = low, relais 5 = low
 // actions trackleg B: relais 3 = high, relais 7 = low
 else if (groenstat01a == HIGH && frogleg_3 == 1){
      relaisPinvalue[1] = HIGH;
      relaisPinvalue[3] = LOW;
      relaisPinvalue[5] = HIGH;
      relaisPinvalue[7] = HIGH;
   // train is going to travel from B0a or B1a to B2d
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[6])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B0a or B1a to B2d");
      }
 // train is going to travel from B0a or B1a to B3d
 // prerequisites: groenstat01a == HIGH en frogleg_3 is right guiding frogleg_3 = 2
 // Active relais are described, NOTE the relais module is driven LOW active 
 // to get a relais up a low level signal is send from de arduino ports
 // actions trackleg A: relais 1 = low, relais 5 = high
 // actions trackleg B: relais 3 = high, relais 7 = high
 else if (groenstat01a == HIGH && frogleg_3 == 2){ 
      relaisPinvalue[1] = HIGH;
      relaisPinvalue[3] = LOW;
      relaisPinvalue[5] = LOW;
      relaisPinvalue[7] = LOW;
   // train is going to travel from B0a or B1a to B3d
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[7])));
      Serial.println (printBuffer);
      // Serial.println ("nn from B0a or B1a to B3d");
      }
 else {
      strcpy_P(printBuffer,(char*)pgm_read_word(&(trvlpathStrings[8])));
      Serial.println (printBuffer);
   // Serial.println ("no train for green led");
      }
   }
 groenstat23d = LOW; // HIGH = green signalling light from 2a or 3a to drive into point frogleg_0 going to blocks B0 or B1
 groenstat01a = LOW; // HIGH = green signalling light from 0d of 1d to drive into point frogleg_1 going to blocks B2 or B3
 travelPathready = LOW;
}

void nmerrorhandling(){
// error handling has two modes this is the NORMAL mode >> info to LEDS
// the first one is in normal function, the other in during the diagnose function
// during normal mode there is no PC connected to the UNO Arduino.
// This means error information should be given by the two leds on the board
// Two leds can give only 3 active combination, you can't see blinking dark leds
// Combination A:> LED1 = on, LED2 = off, Combination B:> LED1 = on, LED2 = on,combination C:> LED1 = off, LED2 = on
// By changing the blinking frequncy of the LED's we can get more possibilities to diliver information to the user.
// In the normal situation it is good that the user can see the that the system is alive so in that case the LEDs will
// have a slow (once every two seconds) alternating blinking pattern.
// The Errorleds are controlled by the TimerOne interupt routine.
// That means that the Errorroutine controls the LEDs with global parameters. 
// During interrupt time, the interuptroutine checks these parameters and write this infomation to the leds at the right frequency
// list of errormeassages over the LEDS:
// 1) combination A, LED1 = on, LED2 = off, 2 Hz used for >> groenstat01a
// 2) combination B, LED1 = on, LED2 = on, 2 Hz used for >> groenstat01d
// 3) combination C, LED1 = off, LED2 = on, 2 Hz used for >> groenstat23a
// 4) combination A, LED1 = on, LED2 = off, 4 Hz used for >> groelstat23d
// 5) combination B, LED1 = on, LED2 = on, 4 Hz used for >> frogleg_0
// 6) combination C, LED1 = off, LED2 = on, 4 Hz used for >> frogleg_1
// 7) combination A, LED1 = on, LED2 = off, 8 Hz used for >> frogleg_2
// 8) combination B, LED1 = on, LED2 = on, 8 Hz used for >> frogleg_3
// 9) combination C, LED1 = off, LED2 = on, 8 Hz used for >> TBD
// 10) combination A, LED1 = on, LED2 = off, 16 Hz used for >> TBD
// 11) combination B, LED1 = on, LED2 = on, 16 Hz used for >> TBD
// 12) combination C, LED1 = off, LED2 = on, 16 Hz used for >> TBD
// errororigings:> groenstat01a, groenstato1d, groenstat23a, groenstat23d, frogleg_0, frogleg_1, frogleg_2, frogleg_3
// digitalWrite( led4Pin, digitalRead(led4Pin)^1);
Serial.print("nmerrorhandling");
if (testmode == HIGH) {//system is in normal mode
    switch(origin) {
      case 4: // "groenstat01a":
       // 1) combination A LED1 = on, LED2 = off, 2 Hz used for >> groenstat01a
       failArea = "gs01a";
       ledtimerh = 250;
       ledtimerl = 250;
       break;
      case 5: // "groenstat01d":
       // 2) combination B LED1 = on, LED2 = on, 2 Hz used for >> groenstat01d
       failArea = "gs01d";
       ledtimerh = 250;
       ledtimerl = 250;
       break;    
      case 6: // "groenstat23a":
       // 3) combination C LED1 = off, LED2 = on, 2 Hz used for >> groenstat23a
       failArea = "gs23a";
       ledtimerh = 250;
       ledtimerl = 250;
       break; 
      case 7: // "groenstat23d":
       // 4) combination A LED1 = on, LED2 = off, 4 Hz used for >> groelstat23d
       failArea = "gs23d";
       ledtimerh = 125;
       ledtimerl = 125;
       break;   
      case 0: // "frogleg_0":
       // 5) combination B LED1 = on, LED2 = on, 4 Hz used for >> frogleg_0
       failArea = "frogleg_0";
       ledtimerh = 125;
       ledtimerl = 125;
       break;   
      case 1: // "frogleg_1":
       // 6) combination C LED1 = off, LED2 = on, 4 Hz used for >>frogleg_1
       failArea = "frogleg_1";
       ledtimerh = 125;
       ledtimerl = 125;
       break;   
      case 2: // "frogleg_2":
       // 7) combination A LED1 = on, LED2 = off, 8 Hz used for >> frogleg_2
       failArea = "frogleg_2";
       ledtimerh = 62;
       ledtimerl = 62;
       break;   
      case 3: // "frogleg_3":
       failArea = "frogleg_3";
       // 8) combination B LED1 = on, LED2 = on, 8 Hz used for >> frogleg_3
       ledtimerh = 62;
       ledtimerl = 62;
       break;   
    }
    // fast blinking to show something went wrong but not one of the above reasons
    failArea = "unknown";
    ledtimerh = 31;
    ledtimerl = 31;
    }
if (testmode == LOW) 
    {
     // someone did switch on the diagnose mode by sticking the jumper
     diagnosemode(); // make sure the error information can be read at the PC screen 
    }
Serial.println(failArea);
}

void diagnosemode(){
debugAgain:
   Serial.println(testArea);
   if (testPin == LOW && testArea == "init"){  
      // in this part the input capture is done for de test and debug fase
      // send some strings to the terminal for a information request to the user
   for (int i = 0; i < 10; i++) {
       strcpy_P(printBuffer,(char*)pgm_read_word(&(diagnoseStrings[i])));
       Serial.println (printBuffer);
       }
   Serial.println("low & ínit");
   averageReadings();
   greenSignaltest();
   detectFrogposition();
   while (Serial.available()<=0) {
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          }
   recvOneChar();
   showNewData();
      // response should be a number as requested in the above lines
      //
   if (resp1Ok == true) {
     switch (resp1){
        //Serial.write(resp1);
        case 'a':
          testArea = String("pp");
          break;
        case 'b':
          testArea = String("gl");
          break;
        case 'c': 
          testArea = String("mvp");
          break;
        case 'd': 
          testArea = String("mvg");
          break;
        case 'e':
          testArea = String("mom");
          break;
        case 'f': 
          testArea = String("erp"); 
          break;
        case 'g': 
          testArea = String("rch"); 
          break;       
        default: 
          // no good input print lines again
          Serial.println("please respond with a letter");
          goto debugAgain; 
        }
     }
   }  
    // make sure we can get information out of the Arduino box for testing
    // test modus is enabled by a testjumper on the arduino board pin 13 will be GROUNDED by this jumper
   if (testPin == LOW && testArea== "mvp"){

MVPentry: 
      averageReadings();
      greenSignaltest();
      detectFrogposition();
      Serial.print("LOW & mvp");
      Serial.print(frogleg_0);
      Serial.print(frogleg_1);
      Serial.print(frogleg_2);  
      Serial.println(frogleg_3); 
      // position frogleg_1_0    
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[0])));
      Serial.println (printBuffer);
      //Serial.print("pos wsl 1_0: ");
      for (int i = 2; i < countmeasurements +2; i++) {
          Serial.print(sensorValue[0][i]);
          Serial.print(", ") ; // print list sensor0Value [0 tot 9] 
          }      
      Serial.println (".");
      Serial.print("avg0 = ");
      Serial.print(sensorValue[0][0]);
      Serial.print(",");
      Serial.println(frogleg_0);
      // position frogleg_1_1
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[1])));
      Serial.println (printBuffer);
      // Serial.print("pos wsl 1_1: ");
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[1][i]);
          Serial.print(", " ); // print list sensor1Value [] 
          } 
      Serial.println (".");
      Serial.print("avg1 = ");
      Serial.print(sensorValue[1][0]);
      Serial.print(",");
      Serial.println(frogleg_1);
      // position frogleg_2_0
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[2])));
      Serial.println (printBuffer);
      // Serial.println("posn wsl 2_0 ");
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[2][i]);
          Serial.print(", ") ; // print list sensor2Value [] 
          } 
      Serial.println (".");
      Serial.print("avg2 = ");
      Serial.print(sensorValue[2][0]);
      Serial.print(",");
      Serial.println(frogleg_2);
      // position frogleg_2_1
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[3])));
      Serial.println (printBuffer);
      // Serial.println("pos wsl 2_1 ");
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[3][i]);
          Serial.print(", ") ; // print list sensor30Value [] 
          } 
      Serial.println (".");
      Serial.print("avg3 = ");
      Serial.print(sensorValue[3][0]);
      Serial.print(",");
      Serial.println(frogleg_3);
      delay (inforeadtime); // waiting period to show the values at the PC screen
      strcpy_P(printBuffer,(char*)pgm_read_word(&(questionStrings[0])));
      Serial.println (printBuffer);
      // nSerial.println ("mvp do you want to see the next loop j or n");
   while (Serial.available()<=0) {
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          }
   recvOneChar();
   showNewData();
   switch (resp1) {
       case 'j':
        goto MVPentry;
        break;
      case 'n':
        Serial.println("ok dan gaan we verder");
        break;

      default:
        goto debugAgain;
         }
    }
     // make sure we can get information out of the Arduino box for testing
     // test modus is enabled by a testjumper on the arduino board pin 5 will be GROUNDED by this jumper
   if (testPin == LOW && testArea == "mvg"){
      // greenDetect0 combi O0+/O1+ (blok 0d en blok 1d) sv4= 
MVGentry:
      averageReadings();
      greenSignaltest();
      detectFrogposition();
      outGreendetect();
      recvOneChar();
      showNewData();
      switch (resp1) {
       case 'j':
        goto MVGentry;
        break;
      case 'n':
        Serial.println("ok dan gaan we verder");
        break;

      default:
        goto debugAgain;
         }
      }
   if (testPin == LOW && testArea == "pp"){
    PPentry:
      averageReadings();
      greenSignaltest();
      detectFrogposition();
      Serial.print("LOW & pp");
      Serial.print(frogleg_0);
      Serial.print(frogleg_1);
      Serial.print(frogleg_2);  
      Serial.println(frogleg_3);
      // Serial.println(frogleg_0);
      // Serial.println(frogleg_1);
      // Serial.println(frogleg_2);
      // Serial.println(frogleg_3);   
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[0])));
      Serial.print (printBuffer);
      // Serial.print("leg pos frogleg_0 ");
      Serial.print(frogleg_0);
      legpos = frogleg_0;
      posfrogleg();
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[1])));
      Serial.print (printBuffer);
      // Serial.print("leg pos frogleg_1 ");
      Serial.print(frogleg_1);
      legpos = frogleg_1;
      posfrogleg();
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[2])));
      Serial.print (printBuffer);
      // Serial.print("leg pos frogleg_2 ");
      Serial.print(frogleg_2);
      legpos = frogleg_2;
      posfrogleg();
      strcpy_P(printBuffer,(char*)pgm_read_word(&(froglegStrings[3])));
      Serial.print (printBuffer);
      // Serial.print("leg pos Point frogleg_3 ");
      Serial.print(frogleg_3);
      legpos = frogleg_3;
      posfrogleg();
      delay (inforeadtime); // waiting period to show the values at the PC screen 
      strcpy_P(printBuffer,(char*)pgm_read_word(&(questionStrings[0])));
      Serial.print (printBuffer);
      // n
      //Serial.println ("pp do you want to see the next loop j or n");
   while (Serial.available()<=0) {
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          }
   recvOneChar();
   showNewData();
   switch (resp1) {
       case 'j':
        goto PPentry;
        break;
      case 'n':
        Serial.println("ok dan gaan we verder");
        break;

      default:
        goto debugAgain;
      }
    }  
    if (testPin == LOW && testArea == "mom"){
    MOMentry:
      averageReadings();
      greenSignaltest();
      detectFrogposition();
      showavgReadings();
      delay (inforeadtime); // waiting period to show the values at the PC screen 
      strcpy_P(printBuffer,(char*)pgm_read_word(&(questionStrings[0])));
      Serial.println (printBuffer);
      // Serial.println ("mom do you want to see the next loop j or n");
    while (Serial.available()<=0) {
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          }
    recvOneChar();
    showNewData();
    switch (resp1) {
       case 'j':
        goto MOMentry;
        break;
       case 'n':
        Serial.println("ok dan gaan we verder");
        break;

       default:
        goto debugAgain;
       }
   }
   if (testPin == LOW && testArea == "erp") {
       detectFrogposition();
    // by enabling this part of the program one can control the relais structure for debugging
    // the user will be asked to select the source LED signal and the destination track.
    // this routine will set de conditions for the set_relais routine.
    // this routine will be initiated when the test modus is enabled by the jumper A5 to ground 
    destr = "destination track";
    Serial.println("test relais bank");
    Serial.println("input can be: 01a,23a,01d,23d");
    Serial.print("sel src track:");
    source = Serial.read();
    if (source == "01a") {
       groenstat01a = HIGH;
       Serial.println(destr + " can be: 2 or 3");
       Serial.print(destr + ": ");
       dt = Serial.read(); //destinationtrack
       if (dt == "2") { 
          frogleg_3 = 2; // point 3 left leading
          }
      else if (dt == "3"){
          frogleg_3 = 1; // point 3 right leading
          }
       }
    else if (source == "23a") {
      groenstat23a = HIGH;
      Serial.println(destr + " can be: 0 or 1");
      Serial.print(destr + ": ");
      dt = Serial.read(); //destinationtrack
      if (dt == "1") { 
         //frogleg_1 = 2; // point 1 left leading
         }
      else if (dt == "0"){
         // frogleg_1 = 1; // point 1 right leading
         }
      }
    else if (source == "01d") {
       groenstat01d = HIGH;
       Serial.println(destr + " can be: 2 or 3");
       Serial.print(destr + ": ");
       dt = Serial.read(); //destinationtrack
       if (dt =="3") { 
          frogleg_0 = 2; // point 0 left leading
        }
       else if (dt == "2"){
          frogleg_0 = 1; // point 0 right leading
          }
       }
    else if (source == "23d") {
       groenstat23d = HIGH;
       Serial.println(destr + " can be: 0 or 1");
       Serial.print(destr + ": ");
       dt = Serial.read(); //destinationtrack
       if (dt == "0") { 
           frogleg_2 = 2; // point 2 left leading
           }
       else if (dt == "1"){
           frogleg_2 = 1; // point 2 right leading
           }
        }   
    else {
       diagnosemode();
       }
   }
   if (testPin == LOW && testArea == "rch") {
      // write default values into output pins
      for (int m = 0; m<8; m++) {
         Serial.print("relais 0");
         Serial.println(m);
         digitalWrite(relaisPin[m], LOW);
         delay (1000); 
         digitalWrite(relaisPin[m], HIGH);
         delay (10);
         }
   }
   
   testArea = String("init");
}
void outGreendetect(){      
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[0])));
      Serial.println (printBuffer);
       // Serial.println("greenDetect0 combi O0+/O1+ (blok 0d en blok 1d) sv4= ");
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[4][i]);
          Serial.print(", ") ; // print list sensor4Value [] 
          } 
      Serial.println (".");
      Serial.print("avg4 = ");
      Serial.println(sensorValue[4][0]);  
      // greenDetect1 combi O0-/O1- (blok 0a en blok 1a) sv5= 
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[1])));
      Serial.println (printBuffer);
      //Serial.println("greenDetect1 combi O0-/O1- (blok 0a en blok 1a) sv5= ");   
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[5][i]);
          Serial.print(", ") ; // print list sensor5Value [] 
          } 
      Serial.println (".");
      Serial.print("avg5 = ");
      Serial.println(sensorValue[5][0]);
      // greenDetect2 combi O2+/O3+ (blok 2d en blok 3d) sv6= 
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[2])));
      Serial.println (printBuffer);
      //Serial.println("greenDetect2 combi O2+/O3+ (blok 2d en blok 3d) sv6= ");    
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[6][i]);
          Serial.print(", ") ; // print list sensor6Value [] 
          } 
      Serial.println (".");
      Serial.print("avg6 = ");
      Serial.println(sensorValue[6][0]);  
      //greenDetect3 combi O2-/O3- (blok 2a en blok 3a) sv7= 
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[3])));
      Serial.println (printBuffer);
      //Serial.println("greenDetect3 combi O2-/O3- (blok 2a en blok 3a) sv7= ");    
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[7][i]);
          Serial.print(", ") ; // print list sensor7Value [] 
          } 
      Serial.println (".");
      Serial.print("avg7 = ");
      Serial.println(sensorValue[7][0]);
      delay (inforeadtime); // waiting period to show the values at the PC screen 
      strcpy_P(printBuffer,(char*)pgm_read_word(&(questionStrings[0])));
      Serial.println (printBuffer);
      // Serial.println ("mvg do you want to see the next loop j or n");
   while (Serial.available()<=0) {
          // digitalWrite( led4Pin, digitalRead(led4Pin)^1);
          }
}
void showavgReadings(){
      Serial.println("measurement matrix :");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[4])));
      Serial.println (printBuffer);
      // Serial.print("frogleg_0 : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[0][i]);
          Serial.print(", ");
          }
      Serial.println(".");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[5])));
      Serial.println (printBuffer);
      // Serial.print("frogleg_1 : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {  
          Serial.print(sensorValue[1][i]);
          Serial.print(", ");
          }
      Serial.println(".");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[6])));
      Serial.println (printBuffer);
      // Serial.print("frogleg_2 : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[2][i]);
          Serial.print(", ");
          }
      Serial.println(".");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[7])));
      Serial.println (printBuffer);
      // Serial.print("frogleg_3 : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[3][i]);
          Serial.print(", ");
          }
      Serial.println(".");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[0])));
      Serial.println (printBuffer);
      // Serial.print("greenled_0d_1d : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[4][i]);
          Serial.print(", ");
          }
      Serial.println(".");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[1])));
      Serial.println (printBuffer);
      // Serial.print("greenled_0a_1a : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[5][i]);
          Serial.print(", ");
          }
      Serial.println(".");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[2])));
      Serial.println (printBuffer);
      // Serial.print("greenled_2d_3d : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[6][i]);
          Serial.print(", ");
          }
      Serial.println(".");
      strcpy_P(printBuffer,(char*)pgm_read_word(&(rapportStrings[3])));
      Serial.println (printBuffer);
      // Serial.print("greenled_2a_3a : "); 
      for (int i = 2; i < countmeasurements + 2; i++) {
          Serial.print(sensorValue[7][i]);
          Serial.print(", ");
          }
          Serial.println(".");  
}

void posfrogleg() {
 // Serial.println("nico");
 // Serial.println(legpos);
 switch (legpos){
    case 3: 
      Serial.println("analog problem!!, see mvp");
      break;
    case 2: 
      //leftguiding
      Serial.println("leftguiding");
      break;
    case 1: 
      // rightguiding     
      Serial.println("rightguiding");
      break;
    case 0: 
      // still moving
      Serial.println("still moving");
  }
}
