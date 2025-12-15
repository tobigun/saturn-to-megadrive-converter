#include <Arduino.h>
//#include <Adafruit_NeoPixel.h>
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <stdint.h>

#define digitalWriteFast(pin, val)  (val ? sio_hw->gpio_set = (1 << pin) : sio_hw->gpio_clr = (1 << pin))
#define digitalReadFast(pin)        ((1 << pin) & sio_hw->gpio_in)

#define LOW 0
#define HIGH 1

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
#define SELECT_PAUSE 5

union SaturnButtonState {    
    uint16_t value;
    // low: pressed, high: released
    struct {
        uint8_t up      : 1;
        uint8_t down    : 1;
        uint8_t left    : 1;
        uint8_t right   : 1;
        uint8_t start   : 1;
        uint8_t a       : 1;
        uint8_t b       : 1;
        uint8_t c       : 1;
        uint8_t x       : 1;
        uint8_t y       : 1;
        uint8_t z       : 1;
        uint8_t l       : 1;
        uint8_t r       : 1;
    };
};

#ifdef USE_LED
static Adafruit_NeoPixel ledPixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

static volatile uint32_t mdButtonStatePatterns[4][2];

static SaturnButtonState readSaturnController();

#define TIMEOUT_US 800 // 800µs

void __not_in_flash_func(updateMegaDriveController)(uint32_t nowUs) {
    static int8_t patternIndex = 0;
    static uint8_t lastSelectState = 0;
    static uint32_t lastSelectToggleTimeUs = 0;

    uint32_t gpioIn = sio_hw->gpio_in;
    uint8_t selectState = (gpioIn >> PIN_MD_SELECT) & 1;
    if (selectState == lastSelectState) {
        if (nowUs - lastSelectToggleTimeUs < TIMEOUT_US) {
            return;
        }
        patternIndex = 0;
    } else {
        lastSelectState = selectState;
        if (selectState) {
            patternIndex = (patternIndex + 1) % 4;
        }
    }

    sio_hw->gpio_togl = (gpioIn ^ mdButtonStatePatterns[patternIndex][selectState]) & PIN_MD_MASK;
    lastSelectToggleTimeUs = nowUs;
}

void initGpio(uint gpio, bool out) {
    gpio_init(gpio);
    gpio_set_dir(gpio, out);
    gpio_pull_up(gpio);
}

void setup() { 
#ifndef NO_USB
    Serial.begin(115200);
#endif

#ifdef USE_LED
    ledPixel.begin();
    ledPixel.setPixelColor(0, Adafruit_NeoPixel::Color(0, 5, 0));
    ledPixel.show();
#endif

    initGpio(PIN_MD_UP_Z, GPIO_OUT);
    initGpio(PIN_MD_DOWN_Y, GPIO_OUT);
    initGpio(PIN_MD_LEFT_X, GPIO_OUT);
    initGpio(PIN_MD_RIGHT_MODE, GPIO_OUT);
    initGpio(PIN_MD_B_A, GPIO_OUT);
    initGpio(PIN_MD_C_START, GPIO_OUT);
    initGpio(PIN_MD_SELECT, GPIO_IN);
}

void loop() {
    uint32_t nowUs = time_us_32();
    updateMegaDriveController(nowUs);
}

void setup1() { 
	// config Saturn pins
	initGpio(PIN_SATURN_S0, GPIO_OUT);
    initGpio(PIN_SATURN_S1, GPIO_OUT);
    initGpio(PIN_SATURN_D0, GPIO_IN);
    initGpio(PIN_SATURN_D1, GPIO_IN);
    initGpio(PIN_SATURN_D2, GPIO_IN);
    initGpio(PIN_SATURN_D3, GPIO_IN);

    #if 0
    // activate three button mode if R trigger is pushed
    readSaturnController();
    indexMask = saturnPressedButtons.r ? 3 : 7;
    #endif
}

static inline uint32_t writeMegaDrivePins(uint8_t up_z, int8_t down_y, int8_t left_x, int8_t right_mode, int8_t b_a, int8_t c_start) {
    return (c_start << PIN_MD_C_START) |
        (b_a << PIN_MD_B_A) |
        (right_mode << PIN_MD_RIGHT_MODE) |
        (left_x << PIN_MD_LEFT_X) |
        (down_y << PIN_MD_DOWN_Y) |
        (up_z << PIN_MD_UP_Z);
}

static inline void writeMegaDrivePins(SaturnButtonState saturnButtons) {
    mdButtonStatePatterns[0][0] = mdButtonStatePatterns[1][0] = writeMegaDrivePins(saturnButtons.up, saturnButtons.down, LOW, LOW, saturnButtons.a, saturnButtons.start);
    mdButtonStatePatterns[0][1] = mdButtonStatePatterns[1][1] = mdButtonStatePatterns[2][1] = writeMegaDrivePins(saturnButtons.up, saturnButtons.down, saturnButtons.left, saturnButtons.right, saturnButtons.b, saturnButtons.c);
    mdButtonStatePatterns[2][0] = writeMegaDrivePins(LOW, LOW, LOW, LOW, saturnButtons.a, saturnButtons.start);
    mdButtonStatePatterns[3][0] = writeMegaDrivePins(HIGH, HIGH, HIGH, HIGH, saturnButtons.a, saturnButtons.start);
    mdButtonStatePatterns[3][1] = writeMegaDrivePins(saturnButtons.z, saturnButtons.y, saturnButtons.x, saturnButtons.r /*mode*/, saturnButtons.b, saturnButtons.c);
}

void loop1() {
    static SaturnButtonState saturnButtonsLast { .value = 0 }; 
    SaturnButtonState saturnButtons = readSaturnController();
    if (saturnButtons.value != saturnButtonsLast.value) {
        saturnButtonsLast.value = saturnButtons.value;
        writeMegaDrivePins(saturnButtons);
    }
    busy_wait_ms(1);
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
    busy_wait_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    result.up = muxData[0];
    result.down = muxData[1];
    result.left = muxData[2];
    result.right = muxData[3];
    
    writeSaturnMuxAddress(1, 0);
    busy_wait_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    result.b = muxData[0];
    result.c = muxData[1];
    result.a = muxData[2];
    result.start = muxData[3];
    
    writeSaturnMuxAddress(0, 0);
    busy_wait_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    result.z = muxData[0];
    result.y = muxData[1];
    result.x = muxData[2];
    result.r = muxData[3];
    
    writeSaturnMuxAddress(1, 1);
    busy_wait_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    result.l = muxData[3];

    //Serial.printf("Saturn Buttons: %02x\n", saturnButtonsVolatile.value);

    return result;
}
