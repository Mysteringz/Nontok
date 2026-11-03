#include "calibration.h"
#include <EEPROM.h>
#include <stddef.h>
namespace {
constexpr uint32_t MAGIC=0x4e544b33;
struct Record { uint32_t magic;uint16_t version,size;uint32_t generation;core::Profile profile;uint16_t crc; };
static_assert(sizeof(Record)*2<=4096,"Two calibration slots must fit Mega EEPROM");
uint16_t checksum(const Record& r) {
  uint16_t crc=0xffff;
  const uint8_t* bytes=reinterpret_cast<const uint8_t*>(&r);
  for(size_t i=0;i<offsetof(Record,crc);++i) { crc^=bytes[i];for(uint8_t b=0;b<8;++b) crc=(crc>>1)^((crc&1)?0xa001:0); }
  return crc;
}
bool read(uint8_t slot,Record& r) {
  EEPROM.get(slot*sizeof(Record),r);
  return r.magic==MAGIC&&r.version==5&&r.size==sizeof(core::Profile)&&r.crc==checksum(r);
}
uint8_t currentSlot=1;uint32_t generation=0;
}
bool profileLoad(core::Profile& p) {
  Record r;bool found=false;
  for(uint8_t slot=0;slot<2;++slot) if(read(slot,r)&&(!found||int32_t(r.generation-generation)>0)) {
    p=r.profile;generation=r.generation;currentSlot=slot;found=true;
  }
  return found;
}
bool profileSave(const core::Profile& p) {
  Record r={};r.magic=MAGIC;r.version=5;r.size=sizeof(core::Profile);r.generation=generation+1;r.profile=p;r.crc=checksum(r);
  uint8_t slot=1-currentSlot;int address=slot*sizeof(Record);
  // Invalidate destination, write payload, then commit magic. Previous slot survives a power loss.
  for(uint8_t i=0;i<4;++i) EEPROM.update(address+i,0);
  const uint8_t* bytes=reinterpret_cast<const uint8_t*>(&r);
  for(size_t i=4;i<sizeof(Record);++i) EEPROM.update(address+i,bytes[i]);
  for(uint8_t i=0;i<4;++i) EEPROM.update(address+i,bytes[i]);
  uint32_t expected=r.generation;
  // Reuse the write buffer for verification instead of another full Profile on the AVR stack.
  if(!read(slot,r)||r.generation!=expected) return false;
  currentSlot=slot;generation=r.generation;return true;
}
