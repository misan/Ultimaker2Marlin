#ifndef DHT_H
#define DHT_H
 #include "Marlin.h"

#ifdef DHT_ENVIRONMENTAL_SENSOR


// Lifetd straight from Adafruit's library   -- since this firmware isn't set up to use Adruino libraries, I copied it into the source fodler and a few small changes to compile


extern char last_temp  ;
extern byte last_humid ;

//-----------------------------------------------------------------------------------------------------------------

void updateAmbientSensor();


/* DHT library 

MIT license
written by Adafruit Industries
*/

// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22 
#define DHT21 21
#define AM2301 21

class DHT {
 private:
  uint8_t data[6];
  uint8_t _pin, _type, _count;
  boolean read(void);
  unsigned long _lastreadtime;
  boolean firstreading;

 public:
  DHT(uint8_t pin, uint8_t type, uint8_t count=6);
  void begin(void);
  float readTemperature(bool S=false);
  float convertCtoF(float);
  float readHumidity(void);

};


extern DHT dht ;


#endif
#endif
