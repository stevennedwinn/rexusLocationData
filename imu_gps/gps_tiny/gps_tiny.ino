//Connect with pin 14 and 15

#include <TinyGPS++.h>

TinyGPSPlus gps; // create gps object

void setup(){
Serial.begin(57600); // connect serial
Serial.println("The GPS Received Signal:");
Serial3.begin(9600); // connect gps sensor

}
 
void loop(){
    while(Serial3.available()){ // check for gps data
    if(gps.encode(Serial3.read()))// encode gps data
    { 
    
    //Latitude
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(),6);
    
    //Longitude
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(),6); 

    //Altitude
    Serial.print("Altitude: "); 
    Serial.println(gps.altitude.feet());

    //Speed
    Serial.print("Speed: "); 
    Serial.println(gps.speed.mph());

    // Number of satellites connected
    Serial.print("Number of Sats connected: "); 
    Serial.println(gps.satellites.value());

    delay(2000);
    
   }
  }
}
