#pragma once
#include <stdint.h>

namespace core {
// D32/PC5: 128 samples spaced exactly 9 CPU cycles apart; 72 us exposure at 16 MHz.
constexpr uint16_t COLOR_BURST_SAMPLES=128;
constexpr uint16_t COLOR_BURST_CYCLES=COLOR_BURST_SAMPLES*9;
constexpr uint16_t COLOR_MIN_BURSTS=96,COLOR_MAX_BURSTS=256;
constexpr uint32_t COLOR_CPU_HZ=16000000UL;
struct FrequencyWindow { uint16_t edges=0;uint32_t cycles=0; };
inline bool frequencyS2(uint8_t channel) { return channel==1||channel==3; }
inline bool frequencyS3(uint8_t channel) { return channel==1||channel==2; }
struct FrequencyStep { bool filterChanged=false,frameReady=false; };
class FrequencyColor {
 public:
  void begin(uint32_t now) { channel_=0;cycleGood_=true;at_=now;valid_=false; }
  bool ready(uint32_t now) const { return uint32_t(now-at_)>=3000; }
  FrequencyStep update(uint32_t now,const FrequencyWindow& window) {
    FrequencyStep result;
    if(!ready(now)) return result;
    const bool exposure=window.cycles>=uint32_t(COLOR_MIN_BURSTS)*COLOR_BURST_CYCLES&&
      window.cycles<=uint32_t(COLOR_MAX_BURSTS)*COLOR_BURST_CYCLES;
    const bool bounded=window.edges<=COLOR_MAX_BURSTS*(COLOR_BURST_SAMPLES/2);
    // Normalize only observed cycles, excluding interrupt/call gaps. Product fits 32 bits.
    uint32_t normalized=exposure&&bounded?
      (uint32_t(window.edges)*(COLOR_CPU_HZ/100)+window.cycles/2)/window.cycles:0;
    if(!exposure||!bounded||window.edges<2||!normalized||normalized>6000||uint32_t(now-at_)>100000)
      cycleGood_=false;
    pending_[channel_]=normalized<=6000?uint16_t(normalized):0;
    if(++channel_==4) {
      valid_=cycleGood_;
      if(valid_) for(uint8_t i=0;i<4;++i) channels_[i]=pending_[i];
      result.frameReady=true;channel_=0;cycleGood_=true;
    }
    at_=now;result.filterChanged=true;return result;
  }
  uint8_t channel() const { return channel_; }
  bool valid() const { return valid_; }
  uint16_t value(uint8_t i) const { return channels_[i]; }
 private:
  uint32_t at_=0;
  uint16_t pending_[4]={},channels_[4]={};
  uint8_t channel_=0;
  bool cycleGood_=true,valid_=false;
};
}
