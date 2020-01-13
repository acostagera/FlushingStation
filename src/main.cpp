#include <Arduino.h>

// Define Relay pins
const byte flushIn = 8;
const byte flushOut = 7;

// Define Switch pins
const byte Switch_On = 2;
const byte Switch_Flush = 12;

// Variables
const unsigned long minute = 60000;            //Define minute in millisecods
const unsigned long primeInterval = minute * 2UL;   //Define how long the initial prime
const unsigned long flushInterval = minute;         //
const unsigned long finishFlush = minute * 3UL;     //  
const unsigned long flushTime = minute * 180UL;     //
unsigned long previousTime = 0;                     //
unsigned long startTime = 0;                        //
bool flushInState = false;                          //
bool flushOutState = false;   
bool stationStatus = false;                    //


void setup() {
  Serial.begin(9600);

  pinMode(Switch_On, INPUT_PULLUP);
  pinMode(Switch_Flush, INPUT_PULLUP);

  pinMode(flushIn,OUTPUT);
  pinMode(flushOut,OUTPUT);
  digitalWrite(flushIn,flushInState);
  digitalWrite(flushOut,flushOutState);

  // Serial.println(minute);
  // Serial.println(primeInterval);
  // Serial.println(flushInterval);
  // Serial.println(finishFlush);
  // Serial.println(flushTime);
 
}

void loop(){
  int Read_On = digitalRead(Switch_On);
  int Read_Flush = digitalRead(Switch_Flush);
  unsigned long currentTime = millis();
  if (Read_On == HIGH && Read_Flush == LOW){
    // Switch on the ON position
    // Serial.println("ON");
    flushInState = true;
    flushOutState = false;
    digitalWrite(flushIn,flushInState);
    digitalWrite(flushOut,flushOutState);
    stationStatus = false;

  }else if(Read_Flush == HIGH && Read_On == LOW){
    // Switch on the Flush position
    //Serial.println("Flush");
    if (stationStatus == false){
      startTime = currentTime;
      previousTime = currentTime;
      stationStatus = true;
      // Serial.println(startTime);
    }
    
    //Serial.println(currentTime);

    if (previousTime - startTime <= primeInterval) {
      
      flushInState = true;
      flushOutState = false;
      digitalWrite(flushIn,flushInState);
      digitalWrite(flushOut,flushOutState);
      previousTime = currentTime;
      // Serial.println("Priming...");

    } else if ((previousTime - startTime >= primeInterval) && (previousTime - startTime <= flushTime)) {
      //Serial.print("Flushing...");
      //previousTime = currentTime;
      if (currentTime - previousTime >= flushInterval) {
        // Serial.println("Flushing....");
        flushInState = !flushInState;
        flushOutState = !flushOutState;
        digitalWrite(flushIn,flushInState);
        digitalWrite(flushOut,flushOutState);
        previousTime = currentTime;
        // Serial.println(flushInState);
        // Serial.println(flushOutState);

      }
      
    }
    if (currentTime - startTime >= flushTime) {

      if (currentTime - previousTime <= finishFlush) {
        digitalWrite(flushIn,true);
        digitalWrite(flushOut,false);
        // Serial.println("Last Flush");
      } else {
        digitalWrite(flushIn,false);
        digitalWrite(flushOut,false);
        // Serial.println("Done");
      }
    }
  }else{
    // Switch on the Off pos
    //Serial.println("Off");
    digitalWrite(flushIn,false);
    digitalWrite(flushOut,false);
    stationStatus = false;

  }    
}