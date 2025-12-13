#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

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

static Adafruit_NeoPixel ledPixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

static volatile SaturnButtonState saturnButtonsVolatile;

static volatile int8_t nextIndex = -1;
static uint8_t indexMask = 7;

static void readSaturnController();


static inline void writeGenesisPins(uint8_t up_z, int8_t down_y, int8_t left_x, int8_t right_mode, int8_t b_a, int8_t c_start) {
    uint32_t mask = (c_start << PIN_MD_C_START) |
        (b_a << PIN_MD_B_A) |
        (right_mode << PIN_MD_RIGHT_MODE) |
        (left_x << PIN_MD_LEFT_X) |
        (down_y << PIN_MD_DOWN_Y) |
        (up_z << PIN_MD_UP_Z);
    sio_hw->gpio_set = mask;
    sio_hw->gpio_clr = ~mask & PIN_MD_MASK;
}

static inline void writeGenesisPins(uint8_t index, SaturnButtonState saturnButtons) {
    switch (index) {
        case 0: case 2: case 4:
            writeGenesisPins(saturnButtons.up, saturnButtons.down, saturnButtons.left, saturnButtons.right, saturnButtons.b, saturnButtons.c);
            break;
        case 1: case 3:
            writeGenesisPins(saturnButtons.up, saturnButtons.down, LOW, LOW, saturnButtons.a, saturnButtons.start);
            break;
        case 5:
            writeGenesisPins(LOW, LOW, LOW, LOW, saturnButtons.a, saturnButtons.start);
            break;
        case 6:
            writeGenesisPins(saturnButtons.z, saturnButtons.y, saturnButtons.x, saturnButtons.r /*mode*/, saturnButtons.b, saturnButtons.c);
            break;
        case 7:
            writeGenesisPins(HIGH, HIGH, HIGH, HIGH, saturnButtons.a, saturnButtons.start);
            break;
    }
}

void __not_in_flash_func(selectChanged)() {
    uint8_t select = digitalReadFast(PIN_MD_SELECT) != 0;

    #if 0
    if (nextIndex < 0) {
        uint8_t select = digitalRead(PIN_MD_SELECT);
        //Serial.printf("Select: %d\n", select);
        nextIndex = select ? 1 : 0;
    }
    #endif

    SaturnButtonState saturnButtonsLocal = { .value = saturnButtonsVolatile.value };
    //writeGenesisPins(nextIndex, saturnButtonsLocal);
    writeGenesisPins(!select, saturnButtonsLocal);
    nextIndex = (nextIndex + 1) % indexMask;

    gpio_acknowledge_irq(PIN_MD_SELECT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
}

void setup() { 
#ifndef NO_USB
    Serial.begin(115200);
#endif

    ledPixel.begin();
    ledPixel.setPixelColor(0, Adafruit_NeoPixel::Color(5, 0, 0));
    ledPixel.show();

	// config Saturn pins
	pinMode(PIN_SATURN_S0, OUTPUT);
    pinMode(PIN_SATURN_S1, OUTPUT);
    pinMode(PIN_SATURN_D0, INPUT);
    pinMode(PIN_SATURN_D1, INPUT);
    pinMode(PIN_SATURN_D2, INPUT);
    pinMode(PIN_SATURN_D3, INPUT);

    pinMode(PIN_MD_UP_Z, OUTPUT);
    pinMode(PIN_MD_DOWN_Y, OUTPUT);
    pinMode(PIN_MD_LEFT_X, OUTPUT);
    pinMode(PIN_MD_RIGHT_MODE, OUTPUT);
    pinMode(PIN_MD_B_A, OUTPUT);
    pinMode(PIN_MD_C_START, OUTPUT);
    pinMode(PIN_MD_SELECT, INPUT);

    readSaturnController();

    #if 0
    // activate three button mode if R trigger is pushed
    indexMask = saturnPressedButtons.r ? 3 : 7;
    #endif
    
    //attachInterrupt(digitalPinToInterrupt(PIN_MD_SELECT), selectChanged, CHANGE);
    //gpio_set_irq_enabled_with_callback(PIN_MD_SELECT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, selectChanged);
    irq_set_exclusive_handler(IO_IRQ_BANK0, selectChanged);
    gpio_set_irq_enabled(PIN_MD_SELECT, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void loop() {
    readSaturnController();
    sleep_ms(1);
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

static void readSaturnController() {
	uint8_t muxData[4];
        
    writeSaturnMuxAddress(0, 1);
    sleep_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    saturnButtonsVolatile.up = muxData[0];
    saturnButtonsVolatile.down = muxData[1];
    saturnButtonsVolatile.left = muxData[2];
    saturnButtonsVolatile.right = muxData[3];
    
    writeSaturnMuxAddress(1, 0);
    sleep_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    saturnButtonsVolatile.b = muxData[0];
    saturnButtonsVolatile.c = muxData[1];
    saturnButtonsVolatile.a = muxData[2];
    saturnButtonsVolatile.start = muxData[3];
    
    writeSaturnMuxAddress(0, 0);
    sleep_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    saturnButtonsVolatile.z = muxData[0];
    saturnButtonsVolatile.y = muxData[1];
    saturnButtonsVolatile.x = muxData[2];
    saturnButtonsVolatile.r = muxData[3];
    
    writeSaturnMuxAddress(1, 1);
    sleep_us(SELECT_PAUSE);
    readSaturnMuxData(muxData);
    saturnButtonsVolatile.l = muxData[3];

    //Serial.printf("Saturn Buttons: %02x\n", saturnButtonsVolatile.value);
}
