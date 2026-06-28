/**
 * @file Sampler_HNSW.ino
 * @brief High-speed Data Acquisition Firmware for Teensy 3.2 and AD9220
 *
 * Testing Strategy:
 * The core logic for this firmare revolves around COBS encoding/decoding
 * and pin timing.
 * A unit testing strategy for the desktop (using a framework like Unity)
 * would involve mocking digitalReadFast/digitalWriteFast and Serial interfaces.
 *
 * Example desktop test for COBS:
 * void test_cobs_encode(void) {
 *    uint8_t input[] = {0x11, 0x00, 0x22};
 *    uint8_t output[10];
 *    size_t len = cobs_encode(input, 3, output);
 *    // output should be: 0x02, 0x11, 0x02, 0x22, 0x01 (or similar valid COBS framing)
 *    TEST_ASSERT_EQUAL(5, len);
 * }
 */

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

/* -----------------------------------------------------------------------------
 * Pin Configuration
 * -----------------------------------------------------------------------------
 */
#define PIN_POWER_CONTROL 18
#define PIN_SOLENOID 9
#define PIN_CLOCK 22

#define PIN_ADC_BIT1 5
#define PIN_ADC_BIT2 21
#define PIN_ADC_BIT3 20
#define PIN_ADC_BIT4 6
#define PIN_ADC_BIT5 8
#define PIN_ADC_BIT6 7
#define PIN_ADC_BIT7 14
#define PIN_ADC_BIT8 2

/* AD9220 is a 12-bit ADC, but per README only 8 bits are used. */

static void my_init_pins(void) {
    pinMode(PIN_POWER_CONTROL, OUTPUT);
    pinMode(PIN_SOLENOID, OUTPUT);
    pinMode(PIN_CLOCK, OUTPUT);

    digitalWriteFast(PIN_POWER_CONTROL, LOW);
    digitalWriteFast(PIN_SOLENOID, LOW);
    digitalWriteFast(PIN_CLOCK, LOW);

    pinMode(PIN_ADC_BIT1, INPUT);
    pinMode(PIN_ADC_BIT2, INPUT);
    pinMode(PIN_ADC_BIT3, INPUT);
    pinMode(PIN_ADC_BIT4, INPUT);
    pinMode(PIN_ADC_BIT5, INPUT);
    pinMode(PIN_ADC_BIT6, INPUT);
    pinMode(PIN_ADC_BIT7, INPUT);
    pinMode(PIN_ADC_BIT8, INPUT);
}

/* -----------------------------------------------------------------------------
 * COBS Protocol
 * -----------------------------------------------------------------------------
 */
static size_t cobs_encode(const uint8_t* input, size_t length, uint8_t* output) {
    if (!input || !output) return 0;

    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < length) {
        if (input[read_index] == 0) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;
    return write_index;
}

static size_t cobs_decode(const uint8_t* input, size_t length, uint8_t* output) {
    if (!input || !output || length == 0) return 0;

    size_t read_index = 0;
    size_t write_index = 0;
    uint8_t code = 0;
    uint8_t i = 0;

    while (read_index < length) {
        code = input[read_index];
        if (read_index + code > length && code != 1) {
            return 0; /* Error: malformed frame */
        }
        read_index++;
        for (i = 1; i < code; i++) {
            output[write_index++] = input[read_index++];
        }
        if (code != 0xFF && read_index != length) {
            output[write_index++] = 0;
        }
    }
    return write_index;
}

/* -----------------------------------------------------------------------------
 * Acquisition System State
 * -----------------------------------------------------------------------------
 */
#define NUM_DATA_POINTS 3000
#define START_HUNT_TIMEOUT_US 10000

static uint8_t average_noise = 0;
static uint8_t payload_buffer[3 + NUM_DATA_POINTS];
static uint8_t tx_buffer[sizeof(payload_buffer) + (sizeof(payload_buffer) / 254) + 2];

#define RX_BUFFER_SIZE 64
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static size_t rx_index = 0;
static uint8_t raw_match_index = 0;
static const uint8_t expected_command[3] = {0xAA, 0x55, 0x01};

/* Read the 8 pins from the ADC */
static inline uint8_t read_adc_pins(void) __attribute__((always_inline));
static inline uint8_t read_adc_pins(void) {
    uint8_t value = 0;
    value |= (digitalReadFast(PIN_ADC_BIT1) << 0);
    value |= (digitalReadFast(PIN_ADC_BIT2) << 1);
    value |= (digitalReadFast(PIN_ADC_BIT3) << 2);
    value |= (digitalReadFast(PIN_ADC_BIT4) << 3);
    value |= (digitalReadFast(PIN_ADC_BIT5) << 4);
    value |= (digitalReadFast(PIN_ADC_BIT6) << 5);
    value |= (digitalReadFast(PIN_ADC_BIT7) << 6);
    value |= (digitalReadFast(PIN_ADC_BIT8) << 7);
    return value;
}

/* Sample by pulsing clock and reading pins */
static inline uint8_t sample_adc(void) __attribute__((always_inline));
static inline uint8_t sample_adc(void) {
    digitalWriteFast(PIN_CLOCK, HIGH);
    __asm__ volatile("nop\n\tnop\n\t");
    uint8_t val = read_adc_pins();
    digitalWriteFast(PIN_CLOCK, LOW);
    __asm__ volatile("nop\n\tnop\n\t");
    return val;
}

static void trigger_solenoid(void) {
    digitalWriteFast(PIN_SOLENOID, HIGH);
    delay(10);
    digitalWriteFast(PIN_SOLENOID, LOW);
}

static void calibrate_noise(void) {
    uint32_t sum = 0;
    const uint16_t CALIBRATION_SAMPLES = 256;
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += sample_adc();
        delayMicroseconds(10);
    }
    average_noise = (uint8_t)(sum / CALIBRATION_SAMPLES);
}

static void acquire_and_transmit(void) {
    trigger_solenoid();

    elapsedMicros hunt_timer = 0;
    const uint8_t THRESHOLD = average_noise + 5;

    while (hunt_timer < START_HUNT_TIMEOUT_US) {
        if (sample_adc() > THRESHOLD) {
            break;
        }
    }

    payload_buffer[0] = 0xAA;
    payload_buffer[1] = 0x55;
    payload_buffer[2] = 0x02;

    uint8_t* data_ptr = &payload_buffer[3];
    for (uint32_t i = 0; i < NUM_DATA_POINTS; i++) {
        *data_ptr++ = sample_adc();
    }

    size_t encoded_len = cobs_encode(payload_buffer, sizeof(payload_buffer), tx_buffer);
    if (encoded_len > 0 && encoded_len < sizeof(tx_buffer)) {
        tx_buffer[encoded_len] = 0x00;
        Serial2.write(tx_buffer, encoded_len + 1);
    }
}

/* -----------------------------------------------------------------------------
 * Serial Manager
 * -----------------------------------------------------------------------------
 */
static bool process_frame(size_t frame_length) {
    uint8_t decoded[RX_BUFFER_SIZE];
    size_t decoded_len = cobs_decode(rx_buffer, frame_length, decoded);

    if (decoded_len == sizeof(expected_command)) {
        if (memcmp(decoded, expected_command, sizeof(expected_command)) == 0) {
            return true;
        }
    }
    return false;
}

static bool check_for_command(void) {
    bool trigger_received = false;

    while (Serial2.available() > 0) {
        int byte_read = Serial2.read();
        if (byte_read < 0) continue;

        uint8_t b = (uint8_t)byte_read;

        /* Raw sequence detection fallback */
        if (b == expected_command[raw_match_index]) {
            raw_match_index++;
            if (raw_match_index == sizeof(expected_command)) {
                trigger_received = true;
                raw_match_index = 0;
                rx_index = 0;
            }
        } else {
            raw_match_index = (b == expected_command[0]) ? 1 : 0;
        }

        /* COBS frame detection */
        if (b == 0x00) {
            if (rx_index > 0) {
                if (process_frame(rx_index)) {
                    trigger_received = true;
                }
                rx_index = 0;
            }
        } else {
            if (rx_index < RX_BUFFER_SIZE) {
                rx_buffer[rx_index++] = b;
            } else {
                rx_index = 0;
            }
        }
    }
    return trigger_received;
}


void setup(void) {
    my_init_pins();
    Serial2.begin(1000000);

    /* Power on components */
    digitalWriteFast(PIN_POWER_CONTROL, HIGH);
    delay(100);

    trigger_solenoid();
    calibrate_noise();
}

void loop(void) {
    if (check_for_command()) {
        acquire_and_transmit();
    }
}
