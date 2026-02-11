| Byte Index | Field                         | Size (bytes) | Description                | Value     |
|------------|-------------------------------|--------------|----------------------------|-----------|
| 0          | Start / Header                | 1            | Identifies packet type     | `0x01`    |
| 1          | Reserved                      | 1            | Padding / future use       | `0x00`    |
| 2          | Reserved                      | 1            | Padding / future use       | `0x00`    |
| 3          | Reserved                      | 1            | Padding / future use       | `0x00`    |
| 4–7        | Front Left Wheel Current      | 4            | Float, amps                | `<float>` |
| 8–11       | Back Left Wheel Current       | 4            | Float, amps                | `<float>` |
| 12–15      | Front Right Wheel Current     | 4            | Float, amps                | `<float>` |
| 16–19      | Back Right Wheel Current      | 4            | Float, amps                | `<float>` |
| 20–23      | Front Drum Current            | 4            | Float, amps                | `<float>` |
| 24–27      | Back Drum Current             | 4            | Float, amps                | `<float>` |
| 28–31      | Front Actuator Current        | 4            | Float, amps                | `<float>` |
| 32–35      | Back Actuator Current         | 4            | Float, amps                | `<float>` |
| 36–39      | Front Actuator Position       | 4            | Float, calibrated position | `<float>` |
| 40–43      | Back Actuator Position        | 4            | Float, calibrated position | `<float>` |
| 44–47      | Front Left Wheel Temperature  | 4            | Float, degrees C           | `<float>` |
| 48–51      | Back Left Wheel Temperature   | 4            | Float, degrees C           | `<float>` |
| 52-55      | Front Right Wheel Temperature | 4            | Float, degrees C           | `<float>` |
| 56-59      | Back Right Wheel Temperature  | 4            | Float, degrees C           | `<float>` |
| 60-63      | Front Drum Temperature        | 4            | Float, degrees C           | `<float>` |
| 64-67      | Back Drum Temperature         | 4            | Float, degrees C           | `<float>` |
| 68-71      | Main Battery Voltage          | 4            | Float, volts               | `<float>` |
| 72-75      | Aux Battery Voltage           | 4            | Float, amps                | `<float>` |
