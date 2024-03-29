/* 
  Sketch generated by the Arduino IoT Cloud Thing "Untitled 2"
  https://create.arduino.cc/cloud/things/000891a6-b897-4091-b7f4-2c93f3870c24 

  Arduino IoT Cloud Variables description

  The following variables are automatically generated and updated when changes are made to the Thing

  float theta1;
  float theta2;
  float theta3;

  Variables which are marked as READ/WRITE in the Cloud Thing will also have functions
  which are called when their values are changed from the Dashboard.
  These functions are generated with the Thing and added at the end of this sketch.
*/

#include "thingProperties.h"
#include <string.h>
char token[30];


void ReceiveData(void)
{
char token[30];
char  *ptheta1, *ptheta2 , *ptheta3;
int i = 0;
  if(Serial.available() > 0)
  {
    while(Serial.available() > 0)
    {
      token[i++] = Serial.read();
      delay(10);
    }
     Serial.print("token = ");
    Serial.print(token);
    Serial.println("");
    delay(10);
      if(token[0] == '/')
      {
            ptheta1 = strtok(token+1 , ",");
            ptheta2 = strtok (NULL, ",");  
            ptheta3 = strtok(NULL , ";"); 
            theta1 = atof(ptheta1);
            theta2 = atof(ptheta2);
            theta3 = atof(ptheta3);
            Serial.printf("theta1 = %f theta2 = %f theta3 = %f \n" , theta1 , theta2 , theta3);
      }

  }

}



void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(115200);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
  delay(500); 

  // Defined in thingProperties.h
  initProperties();
  
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud connection and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

}

void loop() {
  ReceiveData();  
  ArduinoCloud.update();
  
}
