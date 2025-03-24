#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <math.h>

#ifndef UNUSED
#define UNUSED(arg) (void)(arg)
#endif

float remap1(float v, float a1, float b1, float a2, float b2, bool clamp);
float remap2( long v,  long a1,  long b1, float a2, float b2, bool clamp);

long remap3(float v, float a1, float b1,  long a2,  long b2, bool clamp);
long remap4( long v,  long a1,  long b1,  long a2,  long b2, bool clamp);

void byteToHex(uint8_t byte, char hhex, char lhex); // Passa byte per valore
void nibbleToHex(uint8_t nibble, char hex); // Passa nibble per valore

//void byteToHex(const uint8_t & byte, char & hhex, char & lhex); //uint8_t
//void nibbleToHex(const uint8_t & nibble, char & hex);

float rad2deg(float rad);
float deg2rad(float deg);

class Timer {
public:
  Timer() ;
  Timer(unsigned long delta) ;
  Timer(unsigned long delta, unsigned long time);

  void setup(unsigned long delta);
  void reset(unsigned long time);
  bool check(unsigned long time);

private:
  unsigned long time;
  unsigned long delta;
};


#endif  // UTIL_H