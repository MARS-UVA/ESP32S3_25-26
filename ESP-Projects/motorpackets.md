# Robot Packet Documentation

## 1. Construction Robot

### Control Packet (Motor Speed)
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (valid); Otherwise (invalid). Determines validity. |
| 1 | Header | 1 | 0x00 (Control Packet ID). |
| 2 | Front Left Wheel | 1 | 0x00 (Full speed reverse) to 0xFF (Full speed forward). |
| 3 | Back Left Wheel | 1 | 0x00 (Full speed reverse) to 0xFF (Full speed forward). |
| 4 | Front Right Wheel | 1 | 0x00 (Full speed reverse) to 0xFF (Full speed forward). |
| 5 | Back Right Wheel | 1 | 0x00 (Full speed reverse) to 0xFF (Full speed forward). |
| 6 | Actuator | 1 | 0x00 (Full speed reverse) to 0xFF (Full speed forward). |

### Current & Voltage Readings
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (Start of Frame). |
| 1 | Header | 1 | 0x01 (Current and Voltage Packet ID). |
| 2 | Reserved Bit 1 | 1 | 0x00 (Future Use). |
| 3 | Reserved Bit 2 | 1 | 0x00 (Future Use). |
| 4-7 | Front Left Wheel Current | 4 | Float (Amps). |
| 8-11 | Back Left Wheel Current | 4 | Float (Amps). |
| 12-15 | Front Right Wheel Current | 4 | Float (Amps). |
| 16-19 | Back Right Wheel Current | 4 | Float (Amps). |
| 20-23 | Actuator Current | 4 | Float (Amps). |
| 24-27 | Battery Voltage | 4 | Float (Volts). |

### Temperature Readings
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (valid); Otherwise (invalid). |
| 1 | Header | 1 | 0x02 (Control Packet ID). |
| 2 | Reserved Bit 1 | 1 | 0x00 (Future Use). |
| 3 | Reserved Bit 2 | 1 | 0x00 (Future Use). |
| 4-7 | Front Left Wheel Temp | 4 | Float (C). |
| 8-11 | Back Left Wheel Temp | 4 | Float (C). |
| 12-15 | Front Right Wheel Temp | 4 | Float (C). |
| 16-19 | Back Right Wheel Temp | 4 | Float (C). |

### Actuator Position
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (Start of Frame). |
| 1 | Invalid | 1 | 0xFF (valid); Otherwise (invalid). |
| 2 | Header | 1 | 0x03 (Position Packet ID). |
| 3 | Reserved Bit 1 | 1 | 0x00 (Future Use). |
| 4 | Reserved Bit 2 | 1 | 0x00 (Future Use). |
| 5-8 | Actuator Position | 4 | Float (Calibrated Position). |

---

## 2. Excavation Robot

### Control Packet (Motor Speed)
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (Start of Frame). |
| 1 | Invalid | 1 | 0xFF (valid); Otherwise (invalid). |
| 2 | Header | 1 | 0x04 (Position Packet ID). |
| 3 | Front Left Wheel | 1 | 0x00 to 0xFF (Full speed reverse to forward). |
| 4 | Back Left Wheel | 1 | 0x00 to 0xFF (Full speed reverse to forward). |
| 5 | Front Right Wheel | 1 | 0x00 to 0xFF (Full speed reverse to forward). |
| 6 | Back Right Wheel | 1 | 0x00 to 0xFF (Full speed reverse to forward). |
| 7 | Bucket Ladder Actuator | 1 | 0x00 to 0xFF (Full speed reverse to forward). |
| 8 | Conveyor Belt | 1 | 0x00 to 0xFF (Full speed reverse to forward). |
| 9 | Track Actuators | 1 | 0x00 to 0xFF (Full speed reverse to forward). |

### Current & Voltage Readings
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (Start of Frame). |
| 1 | Invalid | 1 | 0xFF (valid); Otherwise (invalid). |
| 2 | Header | 1 | 0x05 (Position Packet ID). |
| 3 | Reserved Bit 1 | 1 | 0x00 (Future Use). |
| 4 | Reserved Bit 2 | 1 | 0x00 (Future Use). |
| 5-8 | Front Left Current | 4 | Float (Amps). |
| 8-11 | Back Left Current | 4 | Float (Amps). |
| 12-15 | Front Right Current | 4 | Float (Amps). |
| 16-19 | Back Right Current | 4 | Float (Amps). |
| 20-23 | Bucket Ladder Current | 4 | Float (Amps). |
| 24-27 | Conveyor Belt Current | 4 | Float (Amps). |
| 28-31 | Left Track Current | 4 | Float (Amps). |
| 32-35 | Right Track Current | 4 | Float (Amps). |
| 36-39 | Main Battery Voltage | 4 | Float (Volts). |
| 40-43 | Auxiliary Battery Voltage | 4 | Float (Volts). |

### Temperature Readings
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (Start of Frame). |
| 1 | Invalid | 1 | 0xFF (valid); Otherwise (invalid). |
| 2 | Header | 1 | 0x06 (Position Packet ID). |
| 3 | Reserved Bit 1 | 1 | 0x00 (Future Use). |
| 4 | Reserved Bit 2 | 1 | 0x00 (Future Use). |
| 5-8 | Front Left Wheel Temp | 4 | Float (C). |
| 9-11 | Back Left Wheel Temp | 4 | Float (C). |
| 12-15 | Front Right Wheel Temp | 4 | Float (C). |
| 16-19 | Back Right Wheel Temp | 4 | Float (C). |
| 20-23 | Bucket Ladder Actuator Temp | 4 | Float (C). |
| 24-27 | Conveyor Belt Temp | 4 | Float (C). |

### Actuator Position
| Byte Index | Field | Size (bytes) | Value / Description |
| :--- | :--- | :--- | :--- |
| 0 | Start | 1 | 0xFF (Start of Frame). |
| 1 | Invalid | 1 | 0xFF (valid); Otherwise (invalid). |
| 2 | Header | 1 | 0x07 (Position Packet ID). |
| 3 | Reserved Bit 1 | 1 | 0x00 (Future Use). |
| 4 | Reserved Bit 2 | 1 | 0x00 (Future Use). |
| 5-8 | Left Track Position | 4 | Float (Calibrated Position). |
| 9-12 | Right Track Position | 4 | Float (Calibrated Position). |
