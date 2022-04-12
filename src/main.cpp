/*
-----Lipo voltage cutoff with setpoint adjustment-----

This program monitors the voltage of a Li-po battery, and opens a 
relay to protect the Li-po battery from discharing to a level where it may be damaged.

Program runs in normal mode where voltage is monitored.

Two buttons are used to adjust the cutoff voltage and timer delay. 


The program has 4 modes:
  mode 1 - Normal:
      -The program runs a loop that tests the Li-po battery voltage and compare it to a setpoint. 
        Li-po battery voltage is fed via a voltage divider that reduce the voltage to 0-5V to A0 . 
        The voltage divider decide the max allowed voltage(max 30V)
        The voltage readings are smoothed out.
        
    The digital value between 0-1023 on pin A0 is then calculated and matched to display the 
        actual voltage on the Li-po battery
        
      When Li-po voltage is above setpoint, D10 is high, enabling power to 
        the relay (diode in this example).
        
    When the voltage is below the setpoint, an adjustable timer is
    started. When the timer is finished, D10 turns low, shutting
    off the relay connecting the Li-po battery to the load.
        
    If the voltage rise above setpoint before the timer is finished, the timer is reset and the system
        continues as normal with the Li-po battery connected to the load via the relay       
        
      -LCD Upper line: Battery + Actual voltage
        -LCD Lower line: Cutoff voltage
        
    mode 2 -Setpoint UP:
      -button connected to A1. short and long press.
        -short press Increments 0.01V pr press
        -long press faster increments       
             
    mode 3 - Setpoint DOWN:
       -button connected to A2. short and long press.
        -short press decrement 0.01V pr press
        -long press faster decrements
        
    mode4 - Setpoint TIMEDELAY:
        -Press both buttons for 2 seconds to start toggling through preset time delays of 0-second to 15-seconds.
      -Increments 1-second, delay 1000ms
      -LCD Upper line: "Timer delay"
        -LCD Lower line: Timer delay is displayed
        -Information is displayed for 2 seconds after buttons are released
 
  Logical table for button long-presses:
                        |Normal mode  | Increase setpoint   | Decrease setpoint   |Adjust delay
button1 UP:             |0            | 1                   | 0                   | 1
button2 DOWN:           |0            | 0                   | 1                   | 1
                                      
  EEPROM memory location: 
                        |#            | short-press: 1      | short-press: 2      | short-press: #
                        |#            | long-press: 3       | long-press: 4       | long-press: 5
*/
//----Inclusion for I2C Oled----

  #include <Arduino.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 32 // OLED display height, in pixels
  
  // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
  #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//----Inclusion for Voltage reading and Voltage smoothing----
//-----https://www.electroschematics.com/9351/arduino-digital-voltmeter/
  int analogInput = A0;  //set A0 to analogue input
  float vout;       // Calculated voltage from A0
  float vin;        // Adjusted Voltage from Vout based of measured resistor values
  float R1 = 100700.0;  // resistance of R1 (100K) Adjust value to actual measured resistance----R1 = 100000.0
  float R2 = 14790.0;   // resistance of R2 (15K) Adjust value to actual measured resistance-----R2 = 22100.0
  int value = 0;      // Linear value between 0-1023 from voltage divider to A0


  // Define the number of samples to keep track of. The higher the number, the
  // more the readings will be smoothed, but the slower the output will respond to
  // the input. Using a constant rather than a normal variable lets us use this
  // value to determine the size of the readings array.
  const int numReadings = 20;
  
  int readings[numReadings];      // the readings from the analog input
  int readIndex = 0;              // the index of the current reading
  int total = 0;                  // the running total
  int average = 0;                // the average
  

//----Inclusion for digital inputs and outputs----

  const int button1 = 7;  // defines pin 7 connected to Up-button
  const int button2 = 8;  // defines pin 8 connected to Down-button
 
  const int relayPin = 10;  // define pin D10 connected to relay
  float cutoffVolt;  // default value for a 4s Li-Po battery  
  

//----inclusion for delays----

  #include <millisDelay.h>
  millisDelay relayDelay; // the delay object 
  unsigned long DELAY_TIME;  // Initial time delay for Voltage cutoff to activate, unsigned long,,, = 5000 
  unsigned long DELAY_TIME_STEP = 1000;      // Milliseconds increased for each increment of voltage cutoff delay time   
  unsigned long buttonToggleTime = 1000;     // Time between increases in voltage cutoff delay time
  unsigned long buttonTimer = 0;             // used to record the time the button is first pressed.
  unsigned long buttonTime = 1000;           // length of time (in milliseconds) you wish to have the button held down for, to activate the long press function
  unsigned long previousMillis = 0;          // will store last time voltage cutoff was reached

  unsigned long dirButtonTimer = 0;          // used to record the time the direction button is first pressed.
  unsigned long dirbuttonTime = 5000;        // 5000 time before power off when dirButton is pressed

  unsigned long autoOffTimer = 0;            // used to record time from last time throttle was zero
  unsigned long autoOffTime = 60000;         // time until auto off when throttle is zero

  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 100;    // the debounce time; increase if the output flickers
  
//----inclusion for buttons boolean----
  bool delayRunning = false; // true if still waiting for delay to finish
  bool relayState = false; // Initial state of relay
  
  bool buttonActive = false; // will be changed dependant on the state of the button; 
                  //  this will allow the code to detect the first loop 
                  //  after the button has been pressed or released.
  
  bool longPressActive = false;// serve two functions, it will allow the code to stop 
                  //  the long press from activating more than once, and 
                  //  will stop the short press function being activated 
                  //  when we release the button after a long press.
  
  bool button1Active = false; // UP-button for voltage treshold
  bool button2Active = false; // DOWN-button for voltage treshold
  
  bool dirButtonActive = false; //Long press of direction button shut down system  dirButtonActive == true) && (millis() - dirButtonTimer
  bool cutoffVoltActive = true; // true if battery voltage is lower than cutoff voltage

  bool autoOffActive =  false; // true if auto off timer is finished

//----inclusion for DC motor speed controland potmeter smoothing---- 
  // Define the number of samples to keep track of. The higher the number, the
  // more the readings will be smoothed, but the slower the output will respond to
  // the input. Using a constant rather than a normal variable lets us use this
  // value to determine the size of the readings array.
  const int numReadings_pot = 10;
  
  int readings_pot[numReadings];      // the readings from the analog input
  int readIndex_pot = 0;              // the index of the current reading
  int total_pot = 0;                  // the running total
  int average_pot = 0;                // the average 
  
  
  const int in1 = 5; // HIGH/LOW output for motor direction
  
  const int dirButton = 9; //Input for switch to change direction of motors, long press = system off            
  //int lastButtonState = HIGH; // used in debounce detection
  //int dirButtonState;   //High state equals to forward movement, LOW is reverse
  int motorDir = LOW; //HIGH = fwd, LOW = bwd
  
  const int enA = 6; // pwm speed signal to H-bridge drivers
  int pot_pin = A1;
  int pot_low_limit;
  int pot_high_limit;
  
  int output;
  int ssr_value;

//----include EEPROM for storing configuration----
  #include <EEPROMex.h>
  const int maxAllowedWrites = 80;
  const int memBase = 0;
  float cutoffVolt_Float;
  unsigned long delayTime_Long;
  #define EEPROMSizeATmega328 1024

// void dirButtonPush(){
//   //----DC-motor direction and speed control----
//     //----Direction button read----
//   int reading = digitalRead(dirButton); // read the state of the switch into a local variable:
//     //----Avoid debounce----
//   if (reading != lastButtonState) {      // If the switch changed, due to noise or pressing:
//     lastDebounceTime = millis();        // reset the debouncing timer
//   }  
//   if ((millis() - lastDebounceTime) > debounceDelay) {
//       if (reading != dirButtonState) {
//       dirButtonState = reading;
//         if(dirButtonState ==HIGH){ //if(digitalRead(dirButton) ==LOW && ssr_value <= pot_low_limit){     && 25 <= pot_low_limit   
//           motorDir =! motorDir;
//         }
//       }
//     }
//           digitalWrite(in1,motorDir);         
//           // save the reading. Next time through the loop, it'll be the lastButtonState:
//             lastButtonState = reading;
       
//   }

  

void OLED_Initialize(){
    display.clearDisplay();       //Clear the buffer for OLED display
    display.setTextColor(WHITE);  //Set the color, on monocrome displays always use white despite actual display color
    display.setTextSize(1);       //Set the font size
    display.setCursor(0, 0);    
    //display.print("Initialising");
    //delay(5000);
}

//----OLED normal view----
void OLED_Display(){
    OLED_Initialize();            // Call function to clear and initialize OLED display
    display.setCursor(0,0);       // (line position,line number) Set cursor to upper left corner. first number is position on the line, last is the line number
    display.print("Battery ");        
    display.print(vin);  
    display.print(" V ");
    display.setCursor(0, 10);    
    display.print("Cutoff  ");
    display.print(cutoffVolt);
    display.print(" V");
    display.setCursor(0, 20);    
    display.print("Cutoff delay ");
    display.print(DELAY_TIME/1000);
    display.print(" S");
    display.display(); // Show the display buffer on the screen    
}

//----LCD delay view----
void OLED_Delay(){      
    OLED_Initialize();            //Call function to clear and initialize OLED display
    display.setCursor(0,0);       // (line position,line number) Set cursor to upper left corner. first number is position on the line, last is the line number    
    display.setCursor(0, 0);  
    display.print("Timer delay     ");
    display.setCursor(0, 10);    
    display.print(DELAY_TIME/1000); //   
    display.print(" Seconds        ") ;
    display.display(); // Show the display buffer on the screen
}
  
void OLED_Delay_release(){
    //Serial.println("OLED_Delay_release");
    OLED_Initialize();            // Call function to clear and initialize OLED display       
    display.setCursor(0, 0);  
    display.print("Timer delay     ");
    display.setCursor(0, 10);    
    display.print(DELAY_TIME/1000); //    /1000
    display.print(" Seconds        ") ; 
    display.display(); // Show the display buffer on the screen
    delay(2000);    
}

//----LCD delay view----
void OLED_Shutoff(){      
    OLED_Initialize();            //Call function to clear and initialize OLED display
    display.setCursor(0,0);       // (line position,line number) Set cursor to upper left corner. first number is position on the line, last is the line number    
    display.setCursor(0, 0);  
    display.print("Shutting down!   ");
    display.setCursor(0, 10);
    display.print("Hold for ") ;    
    display.print(dirbuttonTime/1000);   
    display.print(" Seconds") ;
    display.display(); // Show the display buffer on the screen
}





  
void setup() {
  EEPROM.setMemPool(memBase, EEPROMSizeATmega328);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  cutoffVolt_Float     = EEPROM.getAddress(sizeof(float));
  delayTime_Long      = EEPROM.getAddress(sizeof(long));
  
  pinMode(enA, OUTPUT);               //Output for PWM motor speed signal
  analogWrite (enA,LOW);              
  pinMode (dirButton, INPUT_PULLUP);  //D10 to input. LOW means button is pressed
  pinMode (in1, OUTPUT);              // output for direction change
  digitalWrite(in1, motorDir);        // set motor diretion to fwd initially

  // pot_limit between 0-1023. Used to adjust throttle position/range
  pot_low_limit = 25;
  pot_high_limit = 250;

  
  pinMode(analogInput, INPUT);      // Set A0 mode to input
  pinMode (button1, INPUT);     // Set D2 mode to input
  pinMode(button2, INPUT);    // Set D3 mode to input
  pinMode (relayPin, OUTPUT);       // set D10 mode to ouput  


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32. Initializes the Oled screen
  
  Serial.begin(9600);             // initialize serial communication

  // initialize all the analogue voltage readings to 0:   
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {       
    readings[thisReading] = 0;
  }

    // initialize all the Potmeter readings to 0:
  for (int thisReading_pot = 0; thisReading_pot < numReadings_pot; thisReading_pot++) {
    readings_pot[thisReading_pot] = 0;
  }
  // manual reset values by holding either buttons during power up.
    if(digitalRead(button1) || digitalRead(button2) == HIGH){
    cutoffVolt = 15.2;
    EEPROM.updateFloat(cutoffVolt_Float, cutoffVolt);
    
    DELAY_TIME = 4000;
    EEPROM.updateLong(delayTime_Long, DELAY_TIME);
  }
}


//----Main program----
void loop(){
  
  //----read EEPROM----
  DELAY_TIME = EEPROM.readLong(delayTime_Long);
  cutoffVolt = EEPROM.readFloat(cutoffVolt_Float);

    // //----DC-motor direction and speed control----
    //     if(digitalRead(dirButton) ==LOW && ssr_value <= pot_low_limit){
          
    //       motorDir = !motorDir;
    //       digitalWrite(in1,motorDir);
    //    }

  //----Potmeter voltage smoothing----   
  total_pot = total_pot - readings_pot[readIndex_pot];            // subtract the last reading: 
  readings_pot[readIndex_pot] = analogRead(pot_pin);              // read from the sensor:  
  total_pot = total_pot + readings_pot[readIndex_pot];            // add the reading to the total:   
  readIndex_pot = readIndex_pot + 1;                              // advance to the next position in the array: 
  if (readIndex_pot >= numReadings_pot) {                         // if we're at the end of the array...    
    readIndex_pot = 0;                                            // ...wrap around to the beginning:
  }  
  average_pot = total_pot / numReadings_pot;                      // calculate the average: 
  //Serial.println(average);                                      // send it to the computer as ASCII digits
  //delay(1);

  

  output = average_pot;
  //Serial.print("analog read pot pin: ");
  //Serial.println(output);
    if (output < pot_low_limit) {
      analogWrite(enA, 0);
      //analogWrite(ssr_pin2, 0); 
    }
    else if (output > pot_high_limit) {
      analogWrite(enA, 220); //230
      //analogWrite(ssr_pin2, 255);    
    }
    else {
      ssr_value = map(output, pot_low_limit, pot_high_limit, 0, 220);   //230   
      analogWrite(enA, ssr_value);
      //analogWrite(ssr_pin2, ssr_value);
      //Serial.print("ssr_value: ");
      //Serial.println(ssr_value);
    }

  //----Voltage smoothing----   
  total = total - readings[readIndex];            // subtract the last reading: 
  readings[readIndex] = analogRead(analogInput);  // read from the sensor:  
  total = total + readings[readIndex];            // add the reading to the total:   
  readIndex = readIndex + 1;                      // advance to the next position in the array: 
  if (readIndex >= numReadings) {                 // if we're at the end of the array...    
    readIndex = 0;                                // ...wrap around to the beginning:
  }  
  average = total / numReadings;                // calculate the average: 
  //Serial.println(average);                      // send it to the computer as ASCII digits
  delay(1);                                     // delay in between reads for stability

  //----mode 1 NORMAL---- 
    
  //value = analogRead(analogInput); // read the value at analog input
  //vout = value * (5./1023.); // convert analogue in to voltage level
  vout = average * (5.04/1023.); // convert analogue in to voltage level
  vin = vout / (R2/(R1+R2)); // Adjustment of Vout based of measured resistor values
   
  OLED_Display();  // Call OLED_Display function and wtrite to OLED screen

 //----Read voltage, set booleans and start/stop timer ----
   if(cutoffVolt > vin){       
    if(cutoffVoltActive == false){
        cutoffVoltActive = true;
        relayDelay.start(DELAY_TIME);
        //Serial.print("cutoffVoltActive: ");
        //Serial.println(cutoffVoltActive);          
    }
   } 

   if(cutoffVolt < vin){       
    if(cutoffVoltActive == true){
        cutoffVoltActive = false;
        //Serial.print("cutoffVoltActive: ");
        //Serial.println(cutoffVoltActive);
    }
        relayState=true;
        relayDelay.stop();
        //Serial.print("relayState: ");
        //Serial.println(relayState);
   }

if (relayDelay.justFinished()){
    //Serial.println("Timer finished");
    relayState = false;
  }   

if(relayState == false) digitalWrite (relayPin, LOW);
  else if(relayState == true) digitalWrite (relayPin, HIGH);




//----Power OFF----
    //---- Power off with direction button long press----
    
  if (digitalRead(dirButton) == LOW) {
    if (dirButtonActive == false) {
      dirButtonActive = true;
      dirButtonTimer = millis();
    }
    //OLED_Shutoff();
  }

  if(digitalRead(dirButton) ==LOW && ssr_value <= pot_low_limit){
    
    motorDir = !motorDir;
    digitalWrite(in1,motorDir);
  }

  if ((dirButtonActive == true) && (millis() - dirButtonTimer > dirbuttonTime)) {    //----dirButton long press detection----      
      //while (digitalRead(dirButton) == LOW){
      digitalWrite(relayPin, LOW);
      //}
  }
  else if (digitalRead(dirButton) == HIGH) {
    dirButtonActive = false;
  } 

  //void dirButtonPush();

    //----Power off if throttle is idle for 1 minute.----
  if(output < pot_low_limit){
    if(autoOffActive == false){
      autoOffActive = true;
      autoOffTimer = millis();
    }
  }
  if((autoOffActive == true) && (millis() - autoOffTimer > autoOffTime)){
    digitalWrite(relayPin, LOW);
  }
  else if (output > pot_low_limit){
    autoOffActive = false;
  }


    
  //----Read buttons and set booleans---- 
  if (digitalRead(button1) == HIGH) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
        button1Active = true;
  }

  if (digitalRead(button2) == HIGH) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
        button2Active = true;
  }


//----Button long press----
  if ((buttonActive == true) && (millis() - buttonTimer > buttonTime) && (longPressActive == false)) {
    longPressActive = true;
  
  //----mode 4 DELAY Button long press----    
    if ((button1Active == true) && (button2Active == true)) {            
      while (digitalRead(button1)&& digitalRead(button2) == HIGH){          
        DELAY_TIME += DELAY_TIME_STEP;        
        delay(buttonToggleTime);
        

        if(DELAY_TIME > 15000){
          DELAY_TIME = 0;
        }        
        if (digitalRead(button1) && digitalRead(button2) == LOW){ // break out of while loop if either

         break;                                  // buttons are released
      }
      EEPROM.updateLong(delayTime_Long, DELAY_TIME);
      //Serial.print("mode 4 DELAY Button long press EEPROM update DELAY_TIME: ");
      //Serial.println(DELAY_TIME);
      OLED_Delay();
              
     }
     
  //Serial.println("long press Delay released");
    OLED_Delay_release();
    
  //----mode 2 -Setpoint UP Button long press----
  }   else if((button1Active == true) && (button2Active == false)) {

      while (digitalRead(button1) == HIGH) {        
        cutoffVolt +=0.02;
        delay(50);
          //Serial.print ("cutoff volt adjusting up long press: ");
          //Serial.println (cutoffVolt);
        OLED_Display();                          // Call LCD write function
        if (digitalRead(button1) == LOW){
          break;
        }          
      }
      if(cutoffVolt >= 20.0){
          cutoffVolt = 20.0;
        }
      //Serial.print ("cutoff volt set to: ");
      //Serial.println (cutoffVolt);
      EEPROM.updateFloat(cutoffVolt_Float, cutoffVolt);
     }
  //----mode 3 DOWN active Button long press----     
     else if((button1Active == false) && (button2Active == true)) {
      while (digitalRead(button2) == HIGH) {        
        cutoffVolt -=0.02;
        delay(50);
         //Serial.print ("cutoff volt adjusting down long press: ");
         //Serial.println (cutoffVolt);
         //Serial.print ("cutoff volt Float set to: ");
         //Serial.println (cutoffVolt_Float);
        OLED_Display();                          // Call LCD write function
        if (digitalRead(button2) == LOW){          
          break;
        }
      }
      if(cutoffVolt <= 0.0){
         cutoffVolt = 0.0;
      }
      //Serial.print ("cutoff volt set to: ");
      //Serial.println (cutoffVolt);
      EEPROM.updateFloat(cutoffVolt_Float, cutoffVolt); 
    } 
  }


 
//----Button short press----
  if ((buttonActive == true) && (digitalRead(button1) == LOW) && (digitalRead(button2) == LOW)) {
    if (longPressActive == true) {
      longPressActive = false;
    } else {
  //----mode 4 DELAY Button short press----
      if ((button1Active == true) && (button2Active == true)) {
        OLED_Delay_release();
    //----mode 2 -Setpoint UP Button short press----    
      } else if ((button1Active == true) && (button2Active == false)) {
        cutoffVolt +=0.01;
        //Serial.print ("cutoff volt adjusting up shortpess: ");
        //Serial.println (cutoffVolt);
          if(cutoffVolt > 20.0){
          cutoffVolt = 20.0;
        }
      //Serial.print ("cutoff volt set to: ");
      //Serial.println (cutoffVolt);
      EEPROM.updateFloat(cutoffVolt_Float, cutoffVolt);
         
     //----mode 2 -Setpoint DOWN Button short press----   
      } else {
        cutoffVolt -=0.01;
        //Serial.print ("cutoff volt adjusting down shortpess: ");
        //Serial.println (cutoffVolt);        
         if(cutoffVolt < 0.0){
         cutoffVolt = 0.0;
      }
      //Serial.print ("cutoff volt set to: ");
      //Serial.println (cutoffVolt);
      EEPROM.updateFloat(cutoffVolt_Float, cutoffVolt); 
      }
    }
    buttonActive = false;
    button1Active = false;
    button2Active = false;

  }  
}


// void OLED_Initialize(){
//     display.clearDisplay();       //Clear the buffer for OLED display
//     display.setTextColor(WHITE);  //Set the color, on monocrome displays always use white despite actual display color
//     display.setTextSize(1);       //Set the font size
//     display.setCursor(0, 0);    
//     //display.print("Initialising");
//     //delay(5000);
// }

// //----OLED normal view----
// void OLED_Display(){
//     OLED_Initialize();            // Call function to clear and initialize OLED display
//     display.setCursor(0,0);       // (line position,line number) Set cursor to upper left corner. first number is position on the line, last is the line number
//     display.print("Battery ");        
//     display.print(vin);  
//     display.print(" V ");
//     display.setCursor(0, 10);    
//     display.print("Cutoff  ");
//     display.print(cutoffVolt);
//     display.print(" V");
//     display.setCursor(0, 20);    
//     display.print("Cutoff delay ");
//     display.print(DELAY_TIME/1000);
//     display.print(" S");
//     display.display(); // Show the display buffer on the screen    
// }

// //----LCD delay view----
// void OLED_Delay(){      
//     OLED_Initialize();            //Call function to clear and initialize OLED display
//     display.setCursor(0,0);       // (line position,line number) Set cursor to upper left corner. first number is position on the line, last is the line number    
//     display.setCursor(0, 0);  
//     display.print("Timer delay     ");
//     display.setCursor(0, 10);    
//     display.print(DELAY_TIME/1000); //   
//     display.print(" Seconds        ") ;
//     display.display(); // Show the display buffer on the screen
// }
  
// void OLED_Delay_release(){
//     //Serial.println("OLED_Delay_release");
//     OLED_Initialize();            // Call function to clear and initialize OLED display       
//     display.setCursor(0, 0);  
//     display.print("Timer delay     ");
//     display.setCursor(0, 10);    
//     display.print(DELAY_TIME/1000); //    /1000
//     display.print(" Seconds        ") ; 
//     display.display(); // Show the display buffer on the screen
//     delay(2000);    
// }

// //----LCD delay view----
// void OLED_Shutoff(){      
//     OLED_Initialize();            //Call function to clear and initialize OLED display
//     display.setCursor(0,0);       // (line position,line number) Set cursor to upper left corner. first number is position on the line, last is the line number    
//     display.setCursor(0, 0);  
//     display.print("Shutting down!   ");
//     display.setCursor(0, 10);
//     display.print("Hold for ") ;    
//     display.print(dirbuttonTime/1000);   
//     display.print(" Seconds") ;
//     display.display(); // Show the display buffer on the screen
// }
//
