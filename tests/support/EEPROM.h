#pragma once
#include <stdint.h>
#include <string.h>
#include <stdexcept>
struct FakeEEPROM {
  uint8_t bytes[4096];int writesUntilFailure=-1;
  FakeEEPROM() { memset(bytes,255,sizeof(bytes)); }
  template<class T> void get(int address,T& value) { memcpy(&value,bytes+address,sizeof(T)); }
  void update(int address,uint8_t value) {
    if(writesUntilFailure==0) throw std::runtime_error("power loss");
    if(writesUntilFailure>0) --writesUntilFailure;
    bytes[address]=value;
  }
};
extern FakeEEPROM EEPROM;
