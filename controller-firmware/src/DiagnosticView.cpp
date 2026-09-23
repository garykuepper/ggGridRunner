#include "DiagnosticView.h"

#include <Wire.h>

namespace {

// Formats "MM:SS" (capped at 99:59) for the uptime field. dst must hold >=6.
void formatMmSs(char* dst, uint32_t ms) {
  uint32_t s = ms / 1000UL;
  uint16_t mm = static_cast<uint16_t>(s / 60UL);
  uint16_t ss = static_cast<uint16_t>(s % 60UL);
  if (mm > 99) mm = 99;
  snprintf(dst, 6, "%02u:%02u", mm, ss);
}

// Renders a face/dpad button as "L[X]" or "L[ ]".
void appendButton(char* dst, char glyph, uint8_t pressed) {
  dst[0] = glyph;
  dst[1] = '[';
  dst[2] = pressed ? 'X' : ' ';
  dst[3] = ']';
  dst[4] = ' ';
}

}  // namespace

DiagnosticView::DiagnosticView()
    : _display(kWidth, kHeight, &Wire, -1) {}

bool DiagnosticView::begin() {
  if (!_display.begin(SSD1306_SWITCHCAPVCC, kI2cAddr)) return false;
  _display.clearDisplay();
  _display.setTextSize(1);
  _display.setTextColor(SSD1306_WHITE);
  _display.display();
  return true;
}

void DiagnosticView::showBoot(const char* msg) {
  _display.clearDisplay();
  _display.setCursor(0, 0);
  _display.println(F("GridRunner ctrl"));
  _display.println(msg);
  _display.display();
}

void DiagnosticView::render(const PS4Interface& ps4,
                            const ControlPacket& lastTx,
                            uint32_t txCount,
                            uint16_t txRateHz,
                            uint32_t uptimeMs) {
  const Gamepad_PS4BT& p = ps4.raw();
  char line[22];

  _display.clearDisplay();
  _display.setCursor(0, 0);

  // Line 0: PS4:OK  BAT:nn%  S:status-hex
  uint16_t batPct = (static_cast<uint16_t>(p.battery) * 100u) / 255u;
  snprintf(line, sizeof(line), "PS4:%s BAT:%02u%% S:%02X",
           ps4.connected() ? "OK" : "--",
           batPct,
           ps4.lastStatus());
  _display.println(line);

  // Line 1: LX:nnn LY:nnn  TX:rate
  snprintf(line, sizeof(line), "LX:%03u LY:%03u  TX:%02u",
           p.l_joystick_x, p.l_joystick_y, txRateHz);
  _display.println(line);

  // Line 2: RX:nnn RY:nnn  PKT:count (5 digits, rolls past 99999)
  snprintf(line, sizeof(line), "RX:%03u RY:%03u  PKT:%lu",
           p.r_joystick_x, p.r_joystick_y,
           static_cast<unsigned long>(txCount % 100000UL));
  _display.println(line);

  // Line 3: L2:nnn R2:nnn  ERR:count
  snprintf(line, sizeof(line), "L2:%03u R2:%03u  ERR:%lu",
           p.l2, p.r2,
           static_cast<unsigned long>(ps4.errCount() % 100000UL));
  _display.println(line);

  // Line 4: face buttons  X[ ] O[ ] T[ ]
  char btnLine[22] = {0};
  appendButton(&btnLine[0],  'X', p.button_x);
  appendButton(&btnLine[5],  'O', p.button_circle);
  appendButton(&btnLine[10], 'T', p.button_triangle);
  btnLine[14] = '\0';
  _display.println(btnLine);

  // Line 5: square + dpad up/down
  char btnLine2[22] = {0};
  appendButton(&btnLine2[0],  'S', p.button_square);
  appendButton(&btnLine2[5],  'U', p.button_up);
  appendButton(&btnLine2[10], 'D', p.button_down);
  btnLine2[14] = '\0';
  _display.println(btnLine2);

  // Line 6: mapped THR/STR being sent
  snprintf(line, sizeof(line), "THR:%04u  STR:%+05d",
           lastTx.throttle, lastTx.steering);
  _display.println(line);

  // Line 7: uptime + version
  char mmss[6];
  formatMmSs(mmss, uptimeMs);
  snprintf(line, sizeof(line), "Up:%s     v0.1", mmss);
  _display.println(line);

  _display.display();
}
