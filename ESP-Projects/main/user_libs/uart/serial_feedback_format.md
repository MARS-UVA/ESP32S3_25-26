# Current/Bus Voltage Feedback

| Byte Index | Field                     | Size (bytes) | Description                  | Value     |
| ---------- | ------------------------- | ------------ | ---------------------------- | --------- |
| 0          | Start Byte                | 1            | Packet start marker          | `0xFF` |
| 1          | Header                    | 1            | Currents & voltage packet ID | `0x01`    |
| 2          | Reserved                  | 1            | Future use                   | `0x00`    |
| 3          | Reserved                  | 1            | Future use                   | `0x00`    |
| 4–7        | Front Left Wheel Current  | 4            | Float, amps                  | `<float>` |
| 8–11       | Back Left Wheel Current   | 4            | Float, amps                  | `<float>` |
| 12–15      | Front Right Wheel Current | 4            | Float, amps                  | `<float>` |
| 16–19      | Back Right Wheel Current  | 4            | Float, amps                  | `<float>` |
| 20–23      | Front Drum Current        | 4            | Float, amps                  | `<float>` |
| 24–27      | Back Drum Current         | 4            | Float, amps                  | `<float>` |
| 28–31      | Front Actuator Current    | 4            | Float, amps                  | `<float>` |
| 32–35      | Back Actuator Current     | 4            | Float, amps                  | `<float>` |
| 36–39      | Main Battery Voltage      | 4            | Float, volts                 | `<float>` |
| 40–43      | Aux Battery Voltage       | 4            | Float, volts                 | `<float>` |

# Temperature Feedback
| Byte Index | Field                         | Size (bytes) | Description           | Value     |
| ---------- | ----------------------------- | ------------ | --------------------- | --------- |
| 0          | Start Byte                    | 1            | Packet start marker   | `0xFF` |
| 1          | Header                        | 1            | Temperature packet ID | `0x02`    |
| 2          | Reserved                      | 1            | Future use            | `0x00`    |
| 3          | Reserved                      | 1            | Future use            | `0x00`    |
| 4–7        | Front Left Wheel Temperature  | 4            | Float, °C             | `<float>` |
| 8–11       | Back Left Wheel Temperature   | 4            | Float, °C             | `<float>` |
| 12–15      | Front Right Wheel Temperature | 4            | Float, °C             | `<float>` |
| 16–19      | Back Right Wheel Temperature  | 4            | Float, °C             | `<float>` |
| 20–23      | Front Drum Temperature        | 4            | Float, °C             | `<float>` |
| 24–27      | Back Drum Temperature         | 4            | Float, °C             | `<float>` |

# Position Feedback
| Byte Index | Field                   | Size (bytes) | Description                | Value     |
| ---------- | ----------------------- | ------------ | -------------------------- | --------- |
| 0          | Start Byte              | 1            | Packet start marker        | `0xFF` |
| 1          | Header                  | 1            | Position packet ID         | `0x03`    |
| 2          | Reserved                | 1            | Future use                 | `0x00`    |
| 3          | Reserved                | 1            | Future use                 | `0x00`    |
| 4–7        | Front Actuator Position | 4            | Float, calibrated position | `<float>` |
| 8–11       | Back Actuator Position  | 4            | Float, calibrated position | `<float>` |
