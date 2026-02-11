| Byte Index | Field             | Size (bytes) | Description                      | Value                                                  |
|------------|-------------------|--------------|----------------------------------|--------------------------------------------------------|
| 0          | Start             | 1            | Start-of-frame marker            | 0xFF                                                   |
| 1          | Header            | 1            | Packet type / command identifier | 0 (indicates motor command)                            |
| 2          | Front Left Motor  | 1            | Commanded speed for motor 1      | 0x00 (Full speed reverse) to 0xFF (Full speed forward) |
| 3          | Back Left Motor   | 1            | Commanded speed for motor 2      | 0x00 (Full speed reverse) to 0xFF (Full speed forward) |
| 4          | Front Right Motor | 1            | Commanded speed for motor 3      | 0x00 (Full speed reverse) to 0xFF (Full speed forward) |
| 5          | Back Right Motor  | 1            | Commanded speed for motor 4      | 0x00 (Full speed reverse) to 0xFF (Full speed forward) |
| 6          | Front Drum        | 1            | Commanded speed for motor 5      | 0x00 (Full speed reverse) to 0xFF (Full speed forward) |
| 7          | Front Arm         | 1            | Commanded speed for motor 6      | 0x00 (Full speed lower) to 0xFF (Full speed raise)     |
| 8          | Back Drum         | 1            | Commanded speed for motor 7      | 0x00 (Full speed reverse) to 0xFF (Full speed forward) |
| 9          | Back Arm          | 1            | Commanded speed for motor 8      | 0x00 (Full speed lower) to 0xFF (Full speed raise)     |
