#include <Pixy2.h>        // Camera library
#include <PIDLoop.h>      // PID controller library for motors
#include <NewPing.h>      // Ultrasonic sensor library

// Define ultrasonic sensor pins and maximum distance
#define TRIGGER_PIN    1   // Arduino pin tied to trigger pin on the sensor
#define ECHO_PIN       9   // Arduino pin tied to echo pin on the sensor
#define MAX_DISTANCE 100   // Maximum distance to ping (in centimeters)

// Motor speeds and related constants
#define FAST    120        // Fast speed for normal movement
#define SLOW    100        // Slower speed for precise maneuvers
#define Inter     50        // Decrement to reduce speed for stopping
#define STOP     0       

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int Path; // What way is it going?

//Sonar calculations
long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2; // Calculation for Ultrasonic Inch /IDK I just copied from google and it works.
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2; // Calculation for Ultrasonic Inch /IDK I just copied from google and it works.
}

#define X_CENTER         (pixy.frameWidth/2)

// Motor control pins
const int motorSpeedPin1 = 3;       // Front Left wheel speed pin
const int motorSpeedPin4 = 6;       // Front Right wheel speed pin
const int motorDirectionPin1 = 4;   // Front Left wheel direction pin
const int motorDirectionPin4 = 7;   // Front Right wheel direction pin


// Function to set motor 1 forward
void setMotor1Forward(unsigned char speed)  {
    digitalWrite(motorDirectionPin1, LOW);
    analogWrite(motorSpeedPin1, speed);
}

// Function to set motor 4 forward
void setMotor4Forward(unsigned char speed) {
  digitalWrite(motorDirectionPin4, LOW);
  analogWrite(motorSpeedPin4, speed);

}

void setMotor4Back(char Speed) { ///<Motor4 Reverse
digitalWrite(motorDirectionPin4,HIGH);
analogWrite(motorSpeedPin4,Speed);
}
void setMotor1Back(char Speed) { ///<Motor1 Reverse
digitalWrite(motorDirectionPin1,HIGH);
analogWrite(motorSpeedPin1,Speed);
}

//Controls the turning of the car, if changed at all test. This gave me a headache.
Pixy2 pixy;
PIDLoop headingLoop(500, 0, 25, false);








void setup()
{
    pinMode(TRIGGER_PIN, OUTPUT);  
    pinMode(ECHO_PIN, INPUT);
    Serial.begin(115200);
    pixy.init();
    pixy.setLamp(0, 0);
    pixy.changeProg("line_tracking");
}


void loop()
{
  int8_t res;
  int32_t error;
  int left, right;
  char buf[96];
  int photo1 = analogRead(A0);
  int photo2 = analogRead(A1);
  int photo3 = analogRead(A2);
  int path;
  int intersectionCount = 0;
  int block;

  if (res&LINE_BARCODE)
 {
  
   pixy.line.barcodes->print();
   // code==0 is our left-turn sign
   if (pixy.line.barcodes->m_code==0)
      block = 1;
  
 }




 // Determine path based on sensor readings
    if (photo1 > 20 && photo2 > 10 && photo3 < 8) {
        Serial.println("Path A");
        path = 1;
    } else if (photo1 > 25 && photo2 < 17 && photo3 < 14) {
        Serial.println("Path D");
        path = 2;
    } else if (photo1 < 10 && photo2 < 8 && photo3 > 10) {
        Serial.println("Path B");
        path = 3;
    } else if (photo1 < 20 && photo2 > 13 && photo3 > 15) {
        Serial.println("Path C");
        path = 4;
    } else {
        path = 0;
    }

    //Ultrasonic wave sending code. create a low frequency to bounce off object.
    long duration, distance;
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration/2) / 29.1; //MATH >:(
    
    //If nothing there infront do nothing but if you want you can test it by uncommenting the following.
    if (distance >= 2000 ) {
    //Serial.println("Out of range");
    int block = 0;
    }
    else {
    //If it does see something do this. Will need to create a way to change paths.
    //Serial.print(distance);
    //Serial.println(" cm");
    int block = 1;
    }

  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getMainFeatures();
 
  //delay the inputed vectors to avoid big jumps in direction. Works without it currently but could be used to fine tune intersections and turns.
  delay(0);
 
  // If error or nothing detected, stop motors
  if (res<=0)
  {
    //stop both wheels
    setMotor1Forward(0);
    setMotor4Forward(0);
    return;
  }

  // We found the vector...
  if (res&LINE_VECTOR)
  {
    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

    //print vectors into serial in arduino ide
    //pixy.line.vectors->print();

    // Perform PID calcs on heading error.
    headingLoop.update(error);

    //Serial.print(headingLoop.m_command);


    // separate heading into left and right wheel velocities.
    right = -headingLoop.m_command;
    left = headingLoop.m_command;

    // If vector is heading away from us (arrow pointing up), things are normal.
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1)
    {
      // ... but slow down a little if intersection is present, so we don't miss it.
      if (pixy.line.vectors->m_flags&LINE_FLAG_INTERSECTION_PRESENT)
      {
        left += Inter;
        right += Inter;
      }
      else // otherwise, go forward. IDK if this even works but good luck if you try to test this.
      {
        left += FAST;
        right += FAST;
      }    
    }
    else  // If the vector is pointing down, or down-ish, we need to go backwards to follow. FYI doesnt work but will go crazy if you remove.
    {
      left += SLOW;
      right += SLOW;  
    }
    


 
    switch (path){
     case 0:  // No Passenger
        if (block == 0) {
            if (intersectionCount <= 3) {
                pixy.line.setNextTurn(0);  // Keep straight
            } else if (intersectionCount == 4) {
                left = STOP;  // Stop
                right = STOP;
            }
        } else if (block == 1) {
            // Execute a 90-degree turn if block equals 1
            pixy.line.setNextTurn(90);  // Turn 90 degrees
        }
        break;
        
        
      case 1:  // Special navigation rules for case 1 "A"
        if (intersectionCount < 2) {
            pixy.line.setNextTurn(0);  // Go straight through the first two intersections
        } else if (intersectionCount == 1) {
            if (block == 1) {
                pixy.line.setNextTurn(-90);  // Turn left if block is detected at the second intersection
                // Next command to turn right after executing the left turn
                pixy.line.setNextTurn(90);
            }
        } else if (intersectionCount == 3) {
            left += STOP;  // Stop at the third intersection
            right += STOP;
        } else if (intersectionCount == 4) {
            pixy.line.setNextTurn(0);  // Follow the line through one more intersection
        } else if (intersectionCount == 5) {
            left += STOP;  // Stop at the last intersection
            right += STOP;
        }
       break;


       case 2:  // Specific rules for case 2 //D
        if (block == 1) {
            // If block is 1, go straight through all intersections
            pixy.line.setNextTurn(0);  // Keep going straight
            if (intersectionCount == 6) {
                left += STOP;  // Stop at the last intersection
                right += STOP;
            }
        } else {
            // Normal case when block is not 1
            if (intersectionCount == 0) {
                pixy.line.setNextTurn(0);  // Go straight through the first intersection
            } else if (intersectionCount == 1) {
                pixy.line.setNextTurn(-90);  // Turn left after the first intersection
            } else if (intersectionCount == 2) {
                pixy.line.setNextTurn(-90);  // Turn left again after the first left
            } else if (intersectionCount == 3) {
                pixy.line.setNextTurn(0);  // Go straight after the two left turns
            } else if (intersectionCount == 4) {
                left += STOP;  // Stop at the last intersection
                right += STOP;
            }
        }
        break;   

        
      case 3:  // Specific rules for case 2 //B
        if (block == 1) {
            // If block is 1, go straight through all intersections
            pixy.line.setNextTurn(0);  // Keep going straight
            if (intersectionCount == 7) {
                left += STOP;  // Stop at the last intersection
                right += STOP;
            }
        } else {
            // Normal case when block is not 1
            if (intersectionCount == 0) {
                pixy.line.setNextTurn(0);  // Go straight through the first intersection
            } else if (intersectionCount == 1) {
                pixy.line.setNextTurn(-90);  // Turn left after the first intersection
            } else if (intersectionCount == 2) {
                pixy.line.setNextTurn(-90);  // Turn left again after the first left
            } else if (intersectionCount == 3) {
                pixy.line.setNextTurn(0);  // Go straight after the two left turns
            } else if (intersectionCount == 5) {
                left += STOP;  // Stop at the last intersection
                right += STOP;
            }
        }
        break;

        case 4:  // Specific rules for case 3
        if (block == 1) {
            // Navigate based on block detection
            if (intersectionCount == 0) {
                pixy.line.setNextTurn(0);  // Go straight through the first intersection
            } else if (intersectionCount == 1) {
                pixy.line.setNextTurn(-90);  // Turn left after the first intersection
            } else if (intersectionCount == 2) {
                pixy.line.setNextTurn(90);  // Turn right after the second intersection
                left += STOP;  // Stop at this intersection
                right += STOP;
            }
        } else {
            // No block detected, normal navigation
            if (intersectionCount < 4) {
                pixy.line.setNextTurn(0);  // Go straight through first three intersections
            } else if (intersectionCount == 4) {
                left += STOP;  // Stop at the fourth intersection
                right += STOP;
            }
        }
        break;
           }
           
     }
    

setMotor1Forward(left);  
setMotor4Forward(right);

  //count intersections crossed
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == res&LINE_INTERSECTION) {
        intersectionCount++;
        Serial.print("Intersection Count: ");
        Serial.println(intersectionCount);
        delay(1000); // delay to avoid counting the same intersection multiple times
      }
    }
  }





//Find the centre of the line and keep it within set bounds. Higher number will make the car bounce side to side. too low number will cause it to do burnouts (Rip back wheels)
 if (pixy.line.vectors->m_x1 > 50)//centre line for pixy is 40
   {
     setMotor1Forward(left);
     setMotor4Back(0);
   }
   if (pixy.line.vectors->m_x1 < 5) 
   {
     setMotor1Back(0);
     setMotor4Forward(right);
   }



}
   
