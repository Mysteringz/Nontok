#pragma once
#include <stdint.h>
#include <cmath>
#include <deque>
#include <sstream>
#include <string>
#define F(x) x
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define A4 58
#define A5 59
#define A6 60
#define A7 61
#define A8 62
#define DEG_TO_RAD 0.017453292519943295
extern uint32_t fakeMs;
extern int pwmValues[80], directionValues[80], pinWrites;
inline uint32_t millis() { return fakeMs; }
inline void pinMode(uint8_t,int) { ++pinWrites; }
inline void analogWrite(uint8_t p,int v) { pwmValues[p]=v; ++pinWrites; }
inline void digitalWrite(uint8_t p,int v) { directionValues[p]=v; ++pinWrites; }
struct Stream {
  std::deque<char> input;
  std::ostringstream output;
  void begin(unsigned long) {}
  int availableForWrite() { return 64; }
  size_t write(const uint8_t* data,size_t count) {
    output.write((const char*)data,count); return count;
  }
  int available() { return input.size(); }
  char read() { char c=input.front(); input.pop_front(); return c; }
  template<class T> void print(T v) { output<<v; }
  void print(float v,int) { output<<v; }
  template<class T> void println(T v) { output<<v<<'\n'; }
  void println() { output<<'\n'; }
};
extern Stream Serial, Serial2, Serial3;
