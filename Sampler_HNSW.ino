/**
 * @file Sampler_HNSW.ino
 * @brief High-speed Data Acquisition Firmware for Teensy 3.2 and AD9220
 *
 * Testing Strategy:
 * The core logic for this firmware revolves around COBS encoding/decoding,
 * DMA hardware transfers, and hardware PWM generation.
 * Desktop unit testing would involve mocking Serial interfaces and Cortex-M
 * registers (e.g. DMA_TCD, FTM0, PDB).
 * Hardware-in-the-loop (HIL) testing is recommended for DMA and Timer verification.
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
#define PIN_CLOCK 22 /* PTC1 - FTM0_CH0 */

/*
 * The AD9220 pins chosen perfectly map to the lower 8 bits of GPIOD on the Teensy 3.2 (K20 CPU).
 * PIN 2  = PTD0 (Bit 0)
 * PIN 14 = PTD1 (Bit 1)
 * PIN 7  = PTD2 (Bit 2)
 * PIN 8  = PTD3 (Bit 3)
 * PIN 6  = PTD4 (Bit 4)
 * PIN 20 = PTD5 (Bit 5)
 * PIN 21 = PTD6 (Bit 6)
 * PIN 5  = PTD7 (Bit 7)
 */
#define PIN_ADC_BIT1 5
#define PIN_ADC_BIT2 21
#define PIN_ADC_BIT3 20
#define PIN_ADC_BIT4 6
#define PIN_ADC_BIT5 8
#define PIN_ADC_BIT6 7
#define PIN_ADC_BIT7 14
#define PIN_ADC_BIT8 2

static void my_init_pins(void) {
    pinMode(PIN_POWER_CONTROL, OUTPUT);
    pinMode(PIN_SOLENOID, OUTPUT);

    digitalWriteFast(PIN_POWER_CONTROL, LOW);
    digitalWriteFast(PIN_SOLENOID, LOW);

    /* ADC Pins as inputs */
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
 * Hardware Timer & DMA setup
 * -----------------------------------------------------------------------------
 */
#define TARGET_SAMPLE_RATE_HZ 2000000
#define NUM_DATA_POINTS 3000

/* Buffer layout: Header (3 bytes) + Payload (3000 bytes) */
static uint8_t payload_buffer[3 + NUM_DATA_POINTS];
static volatile bool dma_transfer_done = false;

static void dma_isr(void) {
    DMA_CINT = 0; /* Clear interrupt flag for channel 0 */
    dma_transfer_done = true;
}

static void init_timer_and_dma(void) {
    /* Enable clocks for DMA and DMAMUX */
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    SIM_SCGC6 |= SIM_SCGC6_FTM0;

    /* 1. Setup FTM0 to generate a 2MHz PWM clock on PIN 22 (PTC1, FTM0_CH0) */
    /* And trigger DMA on FTM0 CH0 overflow/match */

    /* Set Pin 22 (PTC1) to FTM0_CH0 alternate function (ALT4) AND enable PORT DMA */
    /* The K20 requires PORT DMA to auto-clear flags without CPU intervention.
       IRQC(1) = DMA on rising edge (when clock pulse starts) */
    PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_IRQC(1);

    FTM0_SC = 0; /* Disable timer while configuring */
    FTM0_CNT = 0;

    /* Calculate modulo for 2MHz based on F_BUS (typically 48MHz on Teensy 3.2 at 96MHz F_CPU) */
    uint32_t modulo = (F_BUS / TARGET_SAMPLE_RATE_HZ) - 1;
    FTM0_MOD = modulo;

    /* Set channel 0 to Edge-aligned PWM (High-true pulses).
       No FTM interrupts/DMA enabled here; the physical pin toggle triggers PORTC DMA. */
    FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_C0V = (modulo + 1) / 2; /* 50% duty cycle */

    /* 2. Configure DMA Channel 0 */
    /* Set DMAMUX to map PORTC (source 51) to DMA Channel 0 */
    DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG0 = DMAMUX_SOURCE_PORTC | DMAMUX_ENABLE;

    /* Enable interrupt in NVIC for DMA CH 0 */
    NVIC_ENABLE_IRQ(IRQ_DMA_CH0);
    attachInterruptVector(IRQ_DMA_CH0, dma_isr);

    /* Start Timer (System Clock, Prescaler 1) */
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
}

static void start_dma_capture(uint8_t* dest_buffer, uint32_t count) {
    dma_transfer_done = false;

    /* Disable DMA channel while configuring */
    DMA_CERQ = 0;

    /* Source: GPIOD_PDIR (Port D Input Data Register).
       Since our pins map to bits 0-7, we can read the lower 8 bits directly. */
    DMA_TCD0_SADDR = &GPIOD_PDIR;
    DMA_TCD0_SOFF = 0; /* Don't increment source address */
    /* SSIZE = 0 (8-bit), DSIZE = 0 (8-bit) */
    DMA_TCD0_ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);

    /* Destination: our payload buffer */
    DMA_TCD0_DADDR = dest_buffer;
    DMA_TCD0_DOFF = 1; /* Increment destination by 1 byte */

    /* Number of bytes to transfer per request (1 byte) */
    DMA_TCD0_NBYTES_MLNO = 1;

    /* Total number of transfers */
    DMA_TCD0_CITER_ELINKNO = count;
    DMA_TCD0_BITER_ELINKNO = count;

    /* Last destination address adjustment (not needed as we re-setup each time) */
    DMA_TCD0_SLAST = 0;
    DMA_TCD0_DLASTSGA = 0;

    /* Enable Interrupt on Major Completion AND Disable Request on Completion
       (Prevents buffer overrun since the FTM clock is continuous) */
    DMA_TCD0_CSR = DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ;

    /* Enable DMA Request for Channel 0 */
    DMA_SERQ = 0;
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
#define START_HUNT_TIMEOUT_US 10000
#define MAX_SOLENOID_RETRIES 3

static uint8_t average_noise = 0;
static uint8_t tx_buffer[sizeof(payload_buffer) + (sizeof(payload_buffer) / 254) + 2];

#define RX_BUFFER_SIZE 64
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static size_t rx_index = 0;
static uint8_t raw_match_index = 0;
static const uint8_t expected_command[3] = {0xAA, 0x55, 0x01};

/* Read the lower 8 bits of GPIOD directly for hunting phase */
static inline uint8_t read_adc_pins_direct(void) __attribute__((always_inline));
static inline uint8_t read_adc_pins_direct(void) {
    return (uint8_t)(GPIOD_PDIR & 0xFF);
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
        sum += read_adc_pins_direct(); /* Clock is running in background via PWM */
        delayMicroseconds(10);
    }
    average_noise = (uint8_t)(sum / CALIBRATION_SAMPLES);
}

static void acquire_and_transmit(void) {
    uint8_t retries = 0;
    bool signal_found = false;
    const uint8_t THRESHOLD = average_noise + 5;

    /* 1. Retry mechanism for Solenoid triggering */
    while (retries < MAX_SOLENOID_RETRIES && !signal_found) {
        trigger_solenoid();

        elapsedMicros hunt_timer = 0;
        while (hunt_timer < START_HUNT_TIMEOUT_US) {
            if (read_adc_pins_direct() > THRESHOLD) {
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

    /* 3. Signal found, execute precise DMA capture */
    payload_buffer[0] = 0xAA;
    payload_buffer[1] = 0x55;
    payload_buffer[2] = 0x02;

    uint8_t* data_ptr = &payload_buffer[3];

    start_dma_capture(data_ptr, NUM_DATA_POINTS);

    /* Wait for DMA transfer to complete */
    while (!dma_transfer_done) {
        /* CPU is free here. It could do other things, but we block to wait for acquisition. */
    }

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
                /* Reset state */
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
                /* Reset frame builder */
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

    /* Teensy 3.2 Alternate pins for Serial2 per README */
    Serial2.setRX(26);
    Serial2.setTX(31);
    Serial2.begin(1000000);

    /* Power on components */
    digitalWriteFast(PIN_POWER_CONTROL, HIGH);
    delay(100);

    /* Initialize DMA and hardware PWM */
    init_timer_and_dma();

    trigger_solenoid();
    calibrate_noise();
}

void loop(void) {
    if (check_for_command()) {
        acquire_and_transmit();

        /* Instead of blindly flushing the serial buffer (which could destroy the
           start of the next command if it arrives quickly), we simply ensure our
           parser state variables are clean. */
        rx_index = 0;
        raw_match_index = 0;
    }
}
