/**
 * @file Sampler_HNSW.ino
 * @brief High-speed Data Acquisition Firmware for Teensy 3.2 and AD9220
 *
 * Testing Strategy:
 * The core logic revolves around PacketSerial (COBS), DMA hardware transfers,
 * hardware PWM generation, and circular buffer extraction.
 * Desktop unit testing would involve mocking Serial interfaces and Cortex-M
 * registers (e.g. DMA_TCD, FTM0, PDB).
 * Hardware-in-the-loop (HIL) testing is recommended for DMA and Timer verification.
 */

#include <Arduino.h>
#include <PacketSerial.h>
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
#define PIN_STEPPER_DIR 3
#define PIN_STEPPER_STEP 4

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
    pinMode(PIN_STEPPER_DIR, OUTPUT);
    pinMode(PIN_STEPPER_STEP, OUTPUT);

    digitalWriteFast(PIN_POWER_CONTROL, LOW);
    digitalWriteFast(PIN_SOLENOID, LOW);
    digitalWriteFast(PIN_STEPPER_DIR, LOW);
    digitalWriteFast(PIN_STEPPER_STEP, LOW);

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
#define PRE_TRIGGER_POINTS 200
#define POST_TRIGGER_POINTS (NUM_DATA_POINTS - PRE_TRIGGER_POINTS)

/*
 * DMA Circular Buffer.
 * Must be aligned to its size for the Kinetis K20 DMA modulo feature.
 * We use a 4096-byte (4KB) buffer.
 */
#define DMA_BUFFER_SIZE 4096
#define DMA_BUFFER_MASK (DMA_BUFFER_SIZE - 1)
uint8_t dma_ring_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(DMA_BUFFER_SIZE)));

static void init_timer_and_dma(void) {
    /* Enable clocks for DMA and DMAMUX */
    SIM_SCGC7 |= SIM_SCGC7_DMA;
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX;
    SIM_SCGC6 |= SIM_SCGC6_FTM0;

    /* 1. Setup FTM0 to generate a 2MHz PWM clock on PIN 22 (PTC1, FTM0_CH0) */

    /* Set Pin 22 (PTC1) to FTM0_CH0 alternate function (ALT4) AND enable PORT DMA */
    PORTC_PCR1 = PORT_PCR_MUX(4) | PORT_PCR_IRQC(1);

    FTM0_SC = 0; /* Disable timer while configuring */
    FTM0_CNT = 0;

    /* Calculate modulo for 2MHz based on F_BUS */
    uint32_t modulo = (F_BUS / TARGET_SAMPLE_RATE_HZ) - 1;
    FTM0_MOD = modulo;

    /* Set channel 0 to Edge-aligned PWM (High-true pulses) */
    FTM0_C0SC = FTM_CSC_MSB | FTM_CSC_ELSB;
    FTM0_C0V = (modulo + 1) / 2; /* 50% duty cycle */

    /* 2. Configure DMA Channel 0 */
    /* Set DMAMUX to map PORTC (source 51) to DMA Channel 0 */
    DMAMUX0_CHCFG0 = DMAMUX_DISABLE;
    DMAMUX0_CHCFG0 = DMAMUX_SOURCE_PORTC | DMAMUX_ENABLE;

    /* Start Timer (System Clock, Prescaler 1) */
    FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
}

static void start_continuous_dma(void) {
    DMA_CERQ = 0;

    DMA_TCD0_SADDR = &GPIOD_PDIR;
    DMA_TCD0_SOFF = 0;

    /* DMOD = 12 (2^12 = 4096 bytes) for circular destination buffer. */
    DMA_TCD0_ATTR = DMA_TCD_ATTR_SMOD(0) | DMA_TCD_ATTR_SSIZE(0) |
                    DMA_TCD_ATTR_DMOD(12) | DMA_TCD_ATTR_DSIZE(0);

    DMA_TCD0_DADDR = dma_ring_buffer;
    DMA_TCD0_DOFF = 1;
    DMA_TCD0_NBYTES_MLNO = 1;

    /* Continuous background transfer */
    DMA_TCD0_CITER_ELINKNO = DMA_BUFFER_SIZE;
    DMA_TCD0_BITER_ELINKNO = DMA_BUFFER_SIZE;

    DMA_TCD0_SLAST = 0;
    DMA_TCD0_DLASTSGA = -DMA_BUFFER_SIZE;

    DMA_TCD0_CSR = 0;

    DMA_SERQ = 0;
}

static void stop_dma(void) {
    DMA_CERQ = 0;
}

static uint32_t get_dma_write_index(void) {
    return ((uint32_t)DMA_TCD0_DADDR) - ((uint32_t)dma_ring_buffer);
}


/* -----------------------------------------------------------------------------
 * Acquisition System State
 * -----------------------------------------------------------------------------
 */
#define START_HUNT_TIMEOUT_US 50000
#define MAX_SOLENOID_RETRIES 3

static uint8_t average_noise = 0;
static uint8_t payload_buffer[3 + NUM_DATA_POINTS];

PacketSerial myPacketSerial;
volatile bool trigger_received = false;

static void step_motor_full(void) {
    /* Full implementation of a stepper motor step (blocking for safety) */
    for(int i = 0; i < 200; i++) { /* 200 steps = 1 revolution typically */
        digitalWriteFast(PIN_STEPPER_STEP, HIGH);
        delayMicroseconds(500);
        digitalWriteFast(PIN_STEPPER_STEP, LOW);
        delayMicroseconds(500);
    }
}

static void trigger_solenoid(void) {
    digitalWriteFast(PIN_SOLENOID, HIGH);
    delay(10);
    digitalWriteFast(PIN_SOLENOID, LOW);
}

static void calibrate_noise(void) {
    start_continuous_dma();
    delay(10); /* Let DMA fill the buffer with noise */
    stop_dma();

    uint32_t sum = 0;
    const uint16_t CALIBRATION_SAMPLES = 256;
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += dma_ring_buffer[i];
    }
    average_noise = (uint8_t)(sum / CALIBRATION_SAMPLES);
}

static void extract_dma_buffer(uint32_t trigger_index, uint8_t* out_payload) {
    uint32_t start_index = (trigger_index + DMA_BUFFER_SIZE - PRE_TRIGGER_POINTS) & DMA_BUFFER_MASK;

    out_payload[0] = 0xAA;
    out_payload[1] = 0x55;
    out_payload[2] = 0x02;

    uint8_t* data_ptr = &out_payload[3];
    for (uint32_t i = 0; i < NUM_DATA_POINTS; i++) {
        *data_ptr++ = dma_ring_buffer[(start_index + i) & DMA_BUFFER_MASK];
    }
}

static void acquire_and_transmit(void) {
    uint8_t retries = 0;
    bool signal_found = false;

    /* Prevent overflow if average_noise is near 255 */
    uint8_t THRESHOLD = (uint8_t)(average_noise + 5);
    if (average_noise > 250) {
        THRESHOLD = 255;
    }

    uint32_t trigger_idx = 0;

    /* 1. Retry mechanism for Solenoid triggering */
    while (retries < MAX_SOLENOID_RETRIES && !signal_found) {
        start_continuous_dma();
        trigger_solenoid();

        elapsedMicros hunt_timer = 0;
        uint32_t read_idx = get_dma_write_index();

        while (hunt_timer < START_HUNT_TIMEOUT_US) {
            uint32_t current_write_idx = get_dma_write_index();

            while (read_idx != current_write_idx) {
                if (dma_ring_buffer[read_idx] > THRESHOLD) {
                    signal_found = true;
                    trigger_idx = read_idx;
                    break;
                }
                read_idx = (read_idx + 1) & DMA_BUFFER_MASK;
            }
            if (signal_found) break;
        }

        if (!signal_found) {
            stop_dma();
            retries++;
            delay(100);
        }
    }

    /* 2. Handle failure if signal is not found after retries */
    if (!signal_found) {
        payload_buffer[0] = 0xAA;
        payload_buffer[1] = 0x55;
        payload_buffer[2] = 0x05; /* 0x05 indicates Timeout/Error (0x03, 0x04 reserved for Stepper) */

        /* PacketSerial automatically handles encoding and appending the 0x00 boundary */
        myPacketSerial.send(payload_buffer, 3);
        return;
    }

    /* 3. Signal found. Let DMA continue running to capture POST_TRIGGER_POINTS */
    while (1) {
        uint32_t current = get_dma_write_index();
        uint32_t distance_written = (current + DMA_BUFFER_SIZE - trigger_idx) & DMA_BUFFER_MASK;
        if (distance_written >= POST_TRIGGER_POINTS) {
            break;
        }
    }

    stop_dma();

    /* 4. Extract data and transmit */
    extract_dma_buffer(trigger_idx, payload_buffer);

    /* PacketSerial automatically handles encoding and appending the 0x00 boundary */
    myPacketSerial.send(payload_buffer, sizeof(payload_buffer));
}

/* -----------------------------------------------------------------------------
 * Serial Packet Handler
 * -----------------------------------------------------------------------------
 */
void onPacketReceived(const uint8_t* buffer, size_t size) {
    if (size == 3 && buffer[0] == 0xAA && buffer[1] == 0x55) {
        if (buffer[2] == 0x01) {
            trigger_received = true;
        } else if (buffer[2] == 0x03) {
            /* Stepper motor Raise command implementation */
            digitalWriteFast(PIN_STEPPER_DIR, HIGH);
            step_motor_full();

            uint8_t ack[] = {0xAA, 0x55, 0x03};
            myPacketSerial.send(ack, 3);
        } else if (buffer[2] == 0x04) {
            /* Stepper motor Lower command implementation */
            digitalWriteFast(PIN_STEPPER_DIR, LOW);
            step_motor_full();

            uint8_t ack[] = {0xAA, 0x55, 0x04};
            myPacketSerial.send(ack, 3);
        }
    }
}

void setup(void) {
    my_init_pins();

    /* Teensy 3.2 Alternate pins for Serial2 per README */
    Serial2.setRX(26);
    Serial2.setTX(31);
    Serial2.begin(1000000);

    /* Setup PacketSerial */
    myPacketSerial.setStream(&Serial2);
    myPacketSerial.setPacketHandler(&onPacketReceived);

    /* Power on components */
    digitalWriteFast(PIN_POWER_CONTROL, HIGH);
    delay(100);

    /* Initialize DMA and hardware PWM */
    init_timer_and_dma();

    calibrate_noise();
}

void loop(void) {
    /* Process incoming serial data via PacketSerial */
    myPacketSerial.update();

    if (trigger_received) {
        trigger_received = false;

        acquire_and_transmit();

        /*
         * No need to flush manually; PacketSerial will elegantly process
         * any remaining bytes in the stream as new packets on the next update().
         * Overlapped commands won't corrupt the current execution.
         */
    }
}
