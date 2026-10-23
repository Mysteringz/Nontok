#include "frequency_color.h"
#include <cassert>
#include <iostream>
static void frame(core::FrequencyColor& c,uint32_t& us,bool bad=false) {
  // Vary active exposure and unobserved delays independently.
  const uint16_t expected[]={900,2400,600,4000};
  for(uint8_t i=0;i<4;++i) {
    assert(c.channel()==i);
    us+=2999;assert(!c.ready(us));assert(!c.update(us,{}).filterChanged);
    us+=1;assert(c.ready(us));
    core::FrequencyWindow w;w.cycles=i%2?230400:115200;
    w.edges=bad&&i==1?0:uint32_t(expected[i])*w.cycles/160000UL;
    us+=i%2?40000:20000; // Delays do not change the denominator.
    auto step=c.update(us,w);assert(step.filterChanged);assert(step.frameReady==(i==3));
  }
  assert(c.valid()==!bad);
  if(!bad) for(uint8_t i=0;i<4;++i) assert(c.value(i)==expected[i]);
}
int main() {
  assert(!core::frequencyS2(0)&&!core::frequencyS3(0));
  assert(core::frequencyS2(1)&&core::frequencyS3(1));
  assert(!core::frequencyS2(2)&&core::frequencyS3(2));
  assert(core::frequencyS2(3)&&!core::frequencyS3(3));
  core::FrequencyColor c;uint32_t us=0xfffff000;c.begin(us);
  frame(c,us); // Unsigned micros wrap during a good acquisition.
  frame(c,us,true); // A missing channel invalidates the entire frame.
  assert(c.value(0)==900&&c.value(1)==2400); // Never publish a partly updated frame.
  frame(c,us); // Recovers on the next complete good frame.
  for(int bad=0;bad<6;++bad) {
    for(int i=0;i<4;++i) {
      core::FrequencyWindow w;w.cycles=160000;w.edges=900;
      if(i==1) {
        if(bad==0) w.cycles=0;
        if(bad==1) w.cycles=core::COLOR_MAX_BURSTS*uint32_t(core::COLOR_BURST_CYCLES)+1;
        if(bad==2) w.edges=1;
        if(bad==3) w.edges=7000; // >600 kHz measured frequency.
        if(bad==4) w.edges=20000; // Impossible physical burst count; no arithmetic wrap.
      }
      us+=bad==5&&i==1?100001:20000;c.update(us,w);
    }
    assert(!c.valid());frame(c,us);assert(c.valid());
  }
  std::cout<<"Frequency colour: filter selection, burst exposure normalization, micros wrap, bad-window rejection and full-frame recovery passed\n";
}
