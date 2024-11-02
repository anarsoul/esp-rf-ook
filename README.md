This is an app for ESP32 to decode the signal from Nexus-TH 433MHz thermal
sensor.

RXB6 RF receiver is connected to GPIO21 (change it in the code if you need a
different pin). RXB6 outputs high level when it detects carrier, low level when
it detects no carrier.

That app uses a busy-loop in main to sample GPIO, using interrupts is not
feasible since you'd need to service at least 4000 interrupts/second to
detect pulses of 500uS.

Nexus-TH uses OOK modulation at 433MHz, basic params:
* pulse is 400-600uS (carrier present)
* preamble is >2000 uS (no carrier)
* end of payload is >3000 uS (no carrier)
* zero is 800-1000 uS (no carrier)
* one is 1650-2150 uS (no carrier)

Signal looks like:
```
   |--|                  |--|       |--|                 |--|                             |--|
   |  |                  |  |       |  |                 |  |                             |  |
---|  |------------------|  |-------|  |-----------------|  |-----------------------------|  |
   PULSE    PREAMBLE     PULSE ZERO PULSE    ONE              END OF PAYLOAD
```

Payload is transmitted several times and consists of 36 bits (preamble and EOP
are not counted)


AAAAAAAA BX CC DDDDDDDDDDDD EEEE FFFFFFFF, where:

* A - ID
* B - 1 if battery is OK, 0 if battery low
* X - always zero
* C - channel, zero based (0 for channel 1)
* D - temperature * 10 in C. E.g. 123 for 12.3C
* E - Unknown
* F - Humidity. Clamp to 100

Create cfg.toml (see cfg.toml.example) to specify your credentials for WiFi and MQTT

The app will publish JSON with temperature and humidity data, example:
```
{"time" : "2024-11-02 12:05:31 UTC", "model" : "Nexus-TH", "id" : 174, "channel" : 1, "battery_ok" : 1, "temperature_C" : 10.100, "humidity" : 91}
```
