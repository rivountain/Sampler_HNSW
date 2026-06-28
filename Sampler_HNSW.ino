/**
 * @file Sampler_HNSW.ino
 * @brief High-speed Data Acquisition Firmware for Teensy 3.2 and AD9220
 *
 * Testing Strategy:
 * The core logic for this firmare revolves around COBS encoding/decoding
 * and strict timing using the ARM Cortex-M DWT Cycle Counter.
 * A unit testing strategy for the desktop (using a framework like Unity)
 * would involve mocking digitalReadFast/digitalWriteFast, Serial interfaces,
 * and ARM_DWT_CYCCNT hardware registers.
 *
 * To test the timeout and retry logic, the mocked sample_adc() would return
 * noise values for the first N retries, and then return a valid signal
 * threshold on the last try to ensure the state machine recovers correctly.
 */

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

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
#define MAX_SOLENOID_RETRIES 3
#define TARGET_SAMPLE_RATE_HZ 2000000

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

/* Unprecise sample mainly used for calibration and hunting */
static inline uint8_t sample_adc_unprecise(void) __attribute__((always_inline));
static inline uint8_t sample_adc_unprecise(void) {
    digitalWriteFast(PIN_CLOCK, HIGH);
    /* Small delay for clock high time */
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
        sum += sample_adc_unprecise();
        delayMicroseconds(10);
    }
    average_noise = (uint8_t)(sum / CALIBRATION_SAMPLES);
}

static void acquire_and_transmit(void) {
    uint8_t retries = 0;
    bool signal_found = false;
    const uint8_t THRESHOLD = average_noise + 5;

    /* Enable DWT Cycle Counter for precise timing */
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    /* 1. Retry mechanism for Solenoid triggering */
    while (retries < MAX_SOLENOID_RETRIES && !signal_found) {
        trigger_solenoid();

        elapsedMicros hunt_timer = 0;
        while (hunt_timer < START_HUNT_TIMEOUT_US) {
            if (sample_adc_unprecise() > THRESHOLD) {
                signal_found = true;
                break;
            }
        }

        if (!signal_found) {
            retries++;
            /* Wait before retrying to let physical mechanics settle */
            delay(100);
        }
    }

    /* 2. Handle failure if signal is not found after retries */
    if (!signal_found) {
        payload_buffer[0] = 0xAA;
        payload_buffer[1] = 0x55;
        payload_buffer[2] = 0x03; /* 0x03 indicates Timeout/Error */

        size_t encoded_len = cobs_encode(payload_buffer, 3, tx_buffer);
        if (encoded_len > 0 && encoded_len < sizeof(tx_buffer)) {
            tx_buffer[encoded_len] = 0x00;
            Serial2.write(tx_buffer, encoded_len + 1);
        }
        return; /* Exit early, do not sample */
    }

    /* 3. Signal found, execute precise strict 2MHz sampling */
    payload_buffer[0] = 0xAA;
    payload_buffer[1] = 0x55;
    payload_buffer[2] = 0x02;

    uint8_t* data_ptr = &payload_buffer[3];
    uint32_t cycles_per_sample = (F_CPU / TARGET_SAMPLE_RATE_HZ);

    /* Disable interrupts to prevent timing jitter */
    noInterrupts();
    uint32_t last_cycle = ARM_DWT_CYCCNT;

    for (uint32_t i = 0; i < NUM_DATA_POINTS; i++) {
        /* Spin-wait using the DWT cycle counter for exact deterministic timing */
        while ((ARM_DWT_CYCCNT - last_cycle) < cycles_per_sample) {
            // Spin
        }
        last_cycle += cycles_per_sample;

        /* Generate clock pulse and read immediately */
        digitalWriteFast(PIN_CLOCK, HIGH);
        *data_ptr++ = read_adc_pins();
        digitalWriteFast(PIN_CLOCK, LOW);
    }

    /* Re-enable interrupts */
    interrupts();

    /* 4. Encode and transmit the data */
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

/* Flush the serial buffer to discard commands received while busy */
static void flush_serial_buffer(void) {
    while (Serial2.available() > 0) {
        Serial2.read();
    }
    rx_index = 0;
    raw_match_index = 0;
}


void setup(void) {
    my_init_pins();

    /* Teensy 3.2 Alternate pins for Serial2 per README */
    Serial2.setRX(26);
    Serial2.setTX(31);
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

        /* Clean and tidy execution: Clear the serial queue so any commands
           sent during the sampling transition are ignored. */
        flush_serial_buffer();
    }
}
