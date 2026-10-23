#pragma once
#include <stdint.h>

namespace core {
// R=00, G=11, B=01, clear=10. Kept independent of Arduino for acquisition tests.
inline bool frequencyS2(uint8_t channel) { return channel==1||channel==3; }
inline bool frequencyS3(uint8_t channel) { return channel==1||channel==2; }
struct FrequencyStep { bool filterChanged=false,frameReady=false; };
class FrequencyColor {
 public:
  void begin(uint32_t now) { channel_=0;gating_=false;cycleGood_=true;at_=now;valid_=false; }
  FrequencyStep update(uint32_t now,uint16_t counter) {
    FrequencyStep result;
    uint32_t elapsed=now-at_; // Unsigned wrap-safe micros arithmetic.
    if(!gating_) {
      if(elapsed>=3000) { at_=now;start_=counter;gating_=true; }
      return result;
    }
    if(elapsed<10000) return result;
    // At <=600 kHz and <=50 ms, fewer than 32768 edges occur, so a 16-bit
    // free-running counter delta remains unambiguous even across one wrap.
    uint16_t edges=uint16_t(counter-start_);
    uint32_t normalized=elapsed<=50000 ? (uint32_t(edges)*10000UL+elapsed/2)/elapsed : 0;
    if(!edges||!normalized||normalized>6000||elapsed>50000) cycleGood_=false;
    pending_[channel_]=normalized<=6000?uint16_t(normalized):0;
    if(++channel_==4) {
      valid_=cycleGood_;
      if(valid_) for(uint8_t i=0;i<4;++i) channels_[i]=pending_[i];
      result.frameReady=true;channel_=0;cycleGood_=true;
    }
    gating_=false;at_=now;result.filterChanged=true;return result;
  }
  uint8_t channel() const { return channel_; }
  bool valid() const { return valid_; }
  uint16_t value(uint8_t i) const { return channels_[i]; }
 private:
  uint32_t at_=0;
  uint16_t start_=0,pending_[4]={},channels_[4]={};
  uint8_t channel_=0;
  bool gating_=false,cycleGood_=true,valid_=false;
};
}
