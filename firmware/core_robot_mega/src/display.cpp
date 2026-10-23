#include "config.h"
#include "display.h"
#if OLED_DRIVER
#include <U8x8lib.h>
#include <Wire.h>
#if OLED_DRIVER == 1
static U8X8_SSD1306_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);
#elif OLED_DRIVER == 3
static U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(U8X8_PIN_NONE);
#else
static U8X8_SH1106_128X64_NONAME_HW_I2C oled(U8X8_PIN_NONE);
#endif
static uint32_t lastDraw = 0;
static bool available = false;
#endif
void displayBegin() {
#if OLED_DRIVER
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(2500, true);
#endif
  Wire.beginTransmission(OLED_ADDRESS);
  if (Wire.endTransmission() != 0) return;
  oled.setI2CAddress(OLED_ADDRESS << 1); oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r); available = true;
#endif
}
void displayUpdate(const core::Controller& controller, const core::Sensors& sensors) {
#if OLED_DRIVER
  if (!available || millis()-lastDraw < 250) return;
  lastDraw = millis();
  oled.clearLine(0); oled.drawString(0,0,core::modeName(controller.mode()));
#if OLED_DRIVER == 3
  oled.clearLine(1);oled.setCursor(0,1);oled.print("L ");oled.print(sensors.range[0]);oled.print(" R ");oled.print(sensors.range[1]);
  oled.clearLine(2);oled.setCursor(0,2);oled.print("Yaw ");oled.print(sensors.heading,1);
  oled.clearLine(3);oled.drawString(0,3,controller.mode()==core::Mode::FAULT?"FAULT: USE STOP":"MEGA CORE");
#else
  oled.clearLine(2); oled.setCursor(0,2); oled.print("Left45 "); oled.print(sensors.range[0]);
  oled.clearLine(3); oled.setCursor(0,3); oled.print("Right45 "); oled.print(sensors.range[1]);

  oled.clearLine(5); oled.setCursor(0,5); oled.print("Yaw "); oled.print(sensors.heading,1);
  oled.clearLine(7); oled.drawString(0,7,controller.mode()==core::Mode::FAULT?"FAULT: USE STOP":"MEGA CORE");
#endif
#else
  (void)controller; (void)sensors;
#endif
}
