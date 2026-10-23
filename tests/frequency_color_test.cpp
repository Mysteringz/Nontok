#include "frequency_color.h"
#include <cassert>
#include <iostream>
static void frame(core::FrequencyColor& c,uint32_t& us,uint16_t& count,bool bad=false) {
  // Counter begins near wrap; elapsed gates deliberately vary due to ultrasonic work.
  const uint16_t expected[]={900,2400,600,4000};
  for(uint8_t i=0;i<4;++i) {
    assert(c.channel()==i);
    us+=2999;assert(!c.update(us,count).filterChanged);
    us+=1;assert(!c.update(us,count).filterChanged); // Capture baseline after filter settling.
    uint32_t gate=i%2?30000:10000;
    us+=gate;count=uint16_t(count+(bad&&i==1?0:expected[i]*(gate/10000)));
    auto step=c.update(us,count);assert(step.filterChanged);assert(step.frameReady==(i==3));
  }
  assert(c.valid()==!bad);
  if(!bad) for(uint8_t i=0;i<4;++i) assert(c.value(i)==expected[i]);
}
int main() {
  assert(!core::frequencyS2(0)&&!core::frequencyS3(0));
  assert(core::frequencyS2(1)&&core::frequencyS3(1));
  assert(!core::frequencyS2(2)&&core::frequencyS3(2));
  assert(core::frequencyS2(3)&&!core::frequencyS3(3));
  core::FrequencyColor c;uint32_t us=0xfffff000;uint16_t counter=65000;c.begin(us);
  frame(c,us,counter); // Both micros and counter wrap during a good acquisition.
  frame(c,us,counter,true); // A missing channel invalidates the entire frame.
  assert(c.value(0)==900&&c.value(1)==2400); // Never publish a partly updated frame.
  frame(c,us,counter); // Recovers on the next complete good frame.
  us+=3000;c.update(us,counter);us+=50001;counter+=100;
  assert(!c.update(us,counter).frameReady);
  // Complete the overlong frame; it must be invalid rather than alias a wrapped counter.
  for(int i=0;i<3;++i) { us+=3000;c.update(us,counter);us+=10000;counter+=100;c.update(us,counter); }
  assert(!c.valid());
  std::cout<<"Frequency colour: filter selection, gating, timing normalization, wraps and invalid-frame recovery passed\n";
}
