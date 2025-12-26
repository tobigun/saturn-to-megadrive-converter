#include "pico/time.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Joystick.h>
#include <USB.h>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <stdint.h>

// https://github.com/libretro-mirrors/mednafen-git/blob/master/src/md/input/gamepad.cpp
// https://github.com/bootsector/smd-sixbutton-encoder/tree/master/src
// https://www.retrosix.wiki/controller-interface-sega-mega-drive
// https://circuit-board.de/forum/index.php/Thread/15187-Saturn-Controller-am-Mega-Drive/

#define PIN_MD_UP_Z 9
#define PIN_MD_DOWN_Y 10
#define PIN_MD_LEFT_X 11
#define PIN_MD_RIGHT_MODE 12
#define PIN_MD_B_A 8
#define PIN_MD_C_START 13
#define PIN_MD_SELECT 7
#define PIN_MD_MASK ((1 << PIN_MD_C_START) | (1 << PIN_MD_B_A) | (1 << PIN_MD_RIGHT_MODE) | (1 << PIN_MD_LEFT_X) | (1 << PIN_MD_DOWN_Y) | (1 << PIN_MD_UP_Z))

#define PIN_SATURN_S0 14
#define PIN_SATURN_S1 15
#define PIN_SATURN_D0 26
#define PIN_SATURN_D1 27
#define PIN_SATURN_D2 28
#define PIN_SATURN_D3 29

// How many microseconds to wait after setting select lines? (2µs is enough according to the Saturn developer's manual)
#define SELECT_PAUSE_US 5

#define TIMEOUT_US 800 // 800µs
#define SATURN_READ_INTERVAL_US 500

union SaturnButtonState {
  uint16_t value;
  // low: pressed, high: released
  struct {
    uint8_t up : 1;
    uint8_t down : 1;
    uint8_t left : 1;
    uint8_t right : 1;
    uint8_t start : 1;
    uint8_t a : 1;
    uint8_t b : 1;
    uint8_t c : 1;
    uint8_t x : 1;
    uint8_t y : 1;
    uint8_t z : 1;
    uint8_t l : 1;
    uint8_t r : 1;
  };
};

static Adafruit_NeoPixel ledPixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

enum ControllerMode {
  MODE_6BUTTON = 0,
  MODE_3BUTTON = 1,
  MODE_USB = 2,
};
static volatile uint8_t mode = MODE_6BUTTON;

static volatile uint8_t selectPending = false;
static volatile int8_t patternIndex = 0;
static volatile uint32_t mdButtonStatePatterns[4][2];

static SaturnButtonState readSaturnController();
static void updateUsb(const SaturnButtonState& saturnButtons);
static void disableUsb();
static ControllerMode detectMode();
static void setLedMode(ControllerMode ledMode);
static void initGpio(uint gpio, bool out);

void __not_in_flash_func(selectChanged)() {
  uint32_t gpioIn = sio_hw->gpio_in;
  uint8_t select = (gpioIn >> PIN_MD_SELECT) & 1;
  if (select && mode == MODE_6BUTTON) {
    patternIndex = (patternIndex + 1) % 4;
  }

  sio_hw->gpio_togl = (gpioIn ^ mdButtonStatePatterns[patternIndex][select]) & PIN_MD_MASK;

  selectPending = true;
  gpio_acknowledge_irq(PIN_MD_SELECT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
}

void checkTimeout(uint32_t nowUs) {
  static uint32_t lastSelectToggleTimeUs = 0;

  if (selectPending) {
    selectPending = false;
    lastSelectToggleTimeUs = nowUs;
    return;
  }

  if (nowUs - lastSelectToggleTimeUs > TIMEOUT_US) {
    uint32_t gpioIn = sio_hw->gpio_in;
    uint8_t select = (gpioIn >> PIN_MD_SELECT) & 1;
    patternIndex = 0;
    sio_hw->gpio_togl = (gpioIn ^ mdButtonStatePatterns[patternIndex][select]) & PIN_MD_MASK;
    lastSelectToggleTimeUs = nowUs;
  }
}

void setup() {
  // Serial.begin(115200);

  ledPixel.begin();
  ledPixel.setPixelColor(0, Adafruit_NeoPixel::Color(5, 0, 0));
  ledPixel.show();

  // config Saturn pins
  initGpio(PIN_SATURN_S0, GPIO_OUT);
  initGpio(PIN_SATURN_S1, GPIO_OUT);
  initGpio(PIN_SATURN_D0, GPIO_IN);
  initGpio(PIN_SATURN_D1, GPIO_IN);
  initGpio(PIN_SATURN_D2, GPIO_IN);
  initGpio(PIN_SATURN_D3, GPIO_IN);

  initGpio(PIN_MD_UP_Z, GPIO_OUT);
  initGpio(PIN_MD_DOWN_Y, GPIO_OUT);
  initGpio(PIN_MD_LEFT_X, GPIO_OUT);
  initGpio(PIN_MD_RIGHT_MODE, GPIO_OUT);
  initGpio(PIN_MD_B_A, GPIO_OUT);
  initGpio(PIN_MD_C_START, GPIO_OUT);
  initGpio(PIN_MD_SELECT, GPIO_IN);

  mode = detectMode();

  if (mode == MODE_USB) {
    Joystick.use8bit(true);
    Joystick.useManualSend(true);
    Joystick.begin();
  } else {
    disableUsb();

    irq_set_exclusive_handler(IO_IRQ_BANK0, selectChanged);
    gpio_set_irq_enabled(PIN_MD_SELECT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
  }

  setLedMode((ControllerMode)mode);
}

// disables USB to avoid interference of the USB timer callback with the IO IRQ
static void disableUsb() {
  // HACK: there is no clean way to disable the timer_task() callback that was registered by USB.begin() with add_alarm_in_us().
  // So we disable the whole alarm IRQ here - we do not need alarms for MegaDrive output.
  uint timerAlarmNum = alarm_pool_timer_alarm_num(alarm_pool_get_default());
  uint irq_num = timer_hardware_alarm_get_irq_num((timer_hw_t*) alarm_pool_get_default_timer(), timerAlarmNum);
  irq_set_enabled(irq_num, false);

  USB.disconnect();
}

static void initGpio(uint gpio, bool out) {
  gpio_init(gpio);
  gpio_set_dir(gpio, out);
  gpio_pull_up(gpio);
}

static inline uint32_t createMDButtonStatePattern(uint8_t up_z, int8_t down_y, int8_t left_x, int8_t right_mode, int8_t b_a, int8_t c_start) {
  return (c_start << PIN_MD_C_START) | (b_a << PIN_MD_B_A) | (right_mode << PIN_MD_RIGHT_MODE) | (left_x << PIN_MD_LEFT_X) | (down_y << PIN_MD_DOWN_Y) | (up_z << PIN_MD_UP_Z);
}

static inline void updateMDButtonStatePatterns(SaturnButtonState saturnButtons) {
  uint32_t defaultLow = createMDButtonStatePattern(saturnButtons.up, saturnButtons.down, LOW, LOW, saturnButtons.a, saturnButtons.start);
  uint32_t defaultHigh = createMDButtonStatePattern(saturnButtons.up, saturnButtons.down, saturnButtons.left, saturnButtons.right, saturnButtons.b, saturnButtons.c);
  mdButtonStatePatterns[0][0] = defaultLow;
  mdButtonStatePatterns[0][1] = defaultHigh;
  mdButtonStatePatterns[1][0] = defaultLow;
  mdButtonStatePatterns[1][1] = defaultHigh;
  mdButtonStatePatterns[2][0] = createMDButtonStatePattern(LOW, LOW, LOW, LOW, saturnButtons.a, saturnButtons.start);
  mdButtonStatePatterns[2][1] = defaultHigh;
  mdButtonStatePatterns[3][0] = createMDButtonStatePattern(HIGH, HIGH, HIGH, HIGH, saturnButtons.a, saturnButtons.start);
  mdButtonStatePatterns[3][1] = createMDButtonStatePattern(saturnButtons.z, saturnButtons.y, saturnButtons.x, saturnButtons.r /*mode*/, saturnButtons.b, saturnButtons.c);
}

void loop() {
  static uint32_t lastControllerReadTimeUs = 0;
  static SaturnButtonState saturnButtonsLast {.value = 0};

  uint32_t nowUs = time_us_32();
  checkTimeout(nowUs);

  if (nowUs - lastControllerReadTimeUs > SATURN_READ_INTERVAL_US) {
    SaturnButtonState saturnButtons = readSaturnController();
    if (saturnButtons.value != saturnButtonsLast.value) {
      saturnButtonsLast = saturnButtons;
      if (mode != MODE_USB) {
        updateMDButtonStatePatterns(saturnButtons);
      } else {
        updateUsb(saturnButtons);
      }
    }
    lastControllerReadTimeUs = nowUs;
  }
}

static void setLedMode(ControllerMode ledMode) {
  switch (ledMode) {
  case MODE_6BUTTON:
    ledPixel.setPixelColor(0, Adafruit_NeoPixel::Color(0, 5, 0));
    break;
  case MODE_3BUTTON:
    ledPixel.setPixelColor(0, Adafruit_NeoPixel::Color(0, 0, 5));
    break;
  case MODE_USB:
    ledPixel.setPixelColor(0, Adafruit_NeoPixel::Color(5, 5, 5));
    break;
  }
  ledPixel.show();
}

static ControllerMode detectMode() {
  while (true) {
    if (tud_connected()) {
      return MODE_USB;
    } else if (digitalReadFast(PIN_MD_SELECT) == LOW) {
      SaturnButtonState saturnPressedButtons = readSaturnController();
      // right trigger released (=HIGH): 6-button mode, else 3-button mode
      return saturnPressedButtons.r != 0 ? MODE_6BUTTON : MODE_3BUTTON;
    }
  }
}

static void updateUsb(const SaturnButtonState& saturnButtons) {
  Joystick.X(saturnButtons.left == 0 ? -127 : (saturnButtons.right == 0 ? 127 : 0));
  Joystick.Y(saturnButtons.up == 0 ? -127 : (saturnButtons.down == 0 ? 127 : 0));
  Joystick.setButton(0, saturnButtons.a == 0);
  Joystick.setButton(1, saturnButtons.b == 0);
  Joystick.setButton(2, saturnButtons.c == 0);
  Joystick.setButton(3, saturnButtons.x == 0);
  Joystick.setButton(4, saturnButtons.y == 0);
  Joystick.setButton(5, saturnButtons.z == 0);
  Joystick.setButton(6, saturnButtons.l == 0);
  Joystick.setButton(7, saturnButtons.r == 0);
  Joystick.setButton(8, saturnButtons.start == 0);
  Joystick.send_now();
}

static void writeSaturnMuxAddress(uint8_t s0, uint8_t s1) {
  digitalWriteFast(PIN_SATURN_S0, s0);
  digitalWriteFast(PIN_SATURN_S1, s1);
}

static void readSaturnMuxData(uint8_t* muxData) {
  // Note: pressed is low, so invert the readings
  uint32_t input = sio_hw->gpio_in;
  muxData[0] = (input >> PIN_SATURN_D0) & 0x1;
  muxData[1] = (input >> PIN_SATURN_D1) & 0x1;
  muxData[2] = (input >> PIN_SATURN_D2) & 0x1;
  muxData[3] = (input >> PIN_SATURN_D3) & 0x1;
}

static SaturnButtonState readSaturnController() {
  uint8_t muxData[4];

  SaturnButtonState result;

  writeSaturnMuxAddress(0, 1);
  busy_wait_us(SELECT_PAUSE_US);
  readSaturnMuxData(muxData);
  result.up = muxData[0];
  result.down = muxData[1];
  result.left = muxData[2];
  result.right = muxData[3];

  writeSaturnMuxAddress(1, 0);
  busy_wait_us(SELECT_PAUSE_US);
  readSaturnMuxData(muxData);
  result.b = muxData[0];
  result.c = muxData[1];
  result.a = muxData[2];
  result.start = muxData[3];

  writeSaturnMuxAddress(0, 0);
  busy_wait_us(SELECT_PAUSE_US);
  readSaturnMuxData(muxData);
  result.z = muxData[0];
  result.y = muxData[1];
  result.x = muxData[2];
  result.r = muxData[3];

  writeSaturnMuxAddress(1, 1);
  busy_wait_us(SELECT_PAUSE_US);
  readSaturnMuxData(muxData);
  result.l = muxData[3];

  // Serial.printf("Saturn Buttons: %02x\n", result.value);
  return result;
}
