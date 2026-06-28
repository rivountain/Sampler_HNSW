# Sampler_HNSW
Data acquisition of HNSW
## Pinouts
|Function|Teensy 3.2 pins|Notes|
|-------- | -------- |-------- |
|Power pin|18|PowerPin controls power to the ADC, OP Amp|
|Solenoid pin|9|Solenoid pin controls the solenoid lift or drop|
|Clock pin|22|Clock pin is the clock signal wire to ADC|
|AD9220 ADC Bit 1|5|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|AD9220 ADC Bit 2|21|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|AD9220 ADC Bit 3|20|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|AD9220 ADC Bit 4|6|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|AD9220 ADC Bit 5|8|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|AD9220 ADC Bit 6|7|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|AD9220 ADC Bit 7|14|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|AD9220 ADC Bit 8|2|AD9220 is a 10-bit ADC, only use 8 bits in this project|
|Serial2 RX|26|Use Serial2 to communicate|
|Serial2 TX|31|Use Serial2 to communicate|
|Serial2 Baud Rate|1000000|Choose high baud rate because our wave is sampled at 2MHz frequency containing 3000 data points within miliseconds|
## Work Flow
1. When Teensy 3.2 being powered on, trigger the solenoid once, checking if current place is fine for the solenoid lift a feromagnet sphere, which is about 4.8mm far from the solenoid.
2. Calculate the average noise value of the ADC output and save it.
3. If detecting a COBS codec 0xAA5501, trigger the solenoid, hunt for the start of the waveform, and then get waveform data points from ADC, encoded with COBS protocal, in which the header is 0xAA5502 following with the data, end with 0x00.
4. Get ready for the next sampling task (waiting for the next AA5501).
