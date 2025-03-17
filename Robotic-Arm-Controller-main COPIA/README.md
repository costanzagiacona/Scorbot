# **Serial Communication**

Serial communication protocol for DC motors control by a microcontroller (Arduino, Raspberry, STM32, ...), supporting the following features:
- Support up to 8 motors (N := numbers of motors)
- The companion PC specify command/operation-mode to the microcontroller.
- The board communicate its state and whether the motors reached theirs endstops.
- Use 8+1 bits for pwms and delta-encoders transmission (the extra bit is for values' sign).
- Allow to setup microcontroller on-board robot parameters.

The communication is always started by the Companion PC (the master), while the Microcontroller (the slave) always reply to a received message.\
Conversely, the Microcontroller never send a message (namely a reply) if not triggered by a message from the Companion PC.\
The exchange of message is eventually timed by the microcontroller (based on the operation mode).

<br><br>

# **Messages**

The **messages** have the following format:

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
|    1    | **Header**: carry command/status code and motor count/index.                            |
|   2-X   | **Payload**: carry data coherent to the code specified in the header byte.              |

There are four types of messages:
- **Control messages** (*ctrl*): used by the master during control operation.
- **Setup messages** (*setup*): used by the master to set slave's parameters.
- **Response messages** (*ack*): used by the slave as acknowledgement.
- **Error messages** (*error*): used by both the master and the slave in case of errors.

<br><br>

## **Header**

The **header** is 1 byte length for both sides of the communication. Let number the bits from the LSb to the MSb (87654321). The header byte has the following format:

| Bits n. | Name | Description                                                                      |
| ------- | ---- | -------------------------------------------------------------------------------- |
|   8-4   | CODE | 5 bits code specifing message type and payload.                                  |
|   3-1   |  NUM | 3 bits unsigned number specifing motor count/index (in respect to code).         |

<br>

The possible **CODE** values are:

| Code | Binary |  Name |  Type | Description                                                       |
| ---- | ------ | ----- | ----- | ----------------------------------------------------------------- |
|    1 |  00001 |  IDLE |  ctrl | set all motors speed to zero.                                     |
|    2 |  00010 |   PWM |  ctrl | directly assign PWMs values.                                      |
|    3 |  00011 |   REF |  ctrl | assign desired encoders values.                                   |
|   16 |  10000 | ROBOT | setup | set robot general parameters (time-sampling).                     |
|   17 |  10001 | MOTOR | setup | set spin/encoder direction and encoder value for motor in header. |
|   18 |  10010 |   PID | setup | sending parameters for PID controller in header.                  |
|   24 |  11000 |  ACKC |   ack | response for control messages.                                    |
|   25 |  11001 |  ACKS |   ack | response for setup messages.                                      |
|   31 |  11111 | ERROR | error | used in case of unexpected/wrong messages.                        |

<br><br>

## **Messages**

Message payload formats from companion PC to microcontroller, for every defined code.

<br>

### ***IDLE***

Payload of 0 bytes -> message of 1 byte.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 00001 (**IDLE**).<br>NUM: number of motors minus one (N-1).                       |

<br>

### ***PWM***

Payload of N+1 bytes -> message of N+2 bytes.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 00010 (**PWM**).<br>NUM: number of motors minus one (N-1).                        |
|    2    | Bit-mask for values' sign [87654321].<br> Bit value: 0 = positive, 1 = negative.        |
|    3    | Unsigned pwm value in [0, 255] for motor 1.                                             |
|   ...   | ...                                                                                     |
|   N+2   | Unsigned pwm value in [0, 255] for motor N.                                             |

<br>

### ***REF***

Payload of N+1 bytes -> message of N+2 bytes.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 00011 (**REF**).<br>NUM: number of motors minus one (N-1).                        |
|    2    | Bit-mask for values' sign [87654321].<br> Bit value: 0 = positive, 1 = negative.        |
|    3    | Unsigned delta-encoder value in [0, 255] for motor 1.                                   |
|   ...   | ...                                                                                     |
|   N+2   | Unsigned delta-encoder value in [0, 255] for motor N.                                   |

<br>

### ***ROBOT***

Payload of 4 bytes -> message of 5 bytes.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 10000 (**ROBOT**).<br>NUM: number of motors minus one (N-1).                      |
|   2-5   | Unsigned timesampling in microseconds resetting value in [0, 2^32 - 1].                 |
|    6    | Unsigned number of allowed control ticks without receiving a new control.               |

Note: the timesampling value is in little-endian notation.  

<br>

### ***MOTOR***

Payload of 5 bytes -> message of 6 bytes.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 10001 (**MOTOR**).<br>NUM: selected motor index in [0, N-1].                      |
|    2    | Motor setup flags (5 bits used).                                                        |
|   3-6   | Signed encoder resetting value [-2^31, 2^31 - 1].                                       |

The setup flags byte has the following format ([87654321]):
- bit 1: specify whether to change encoder count;
- bit 2: specify motor spin direction;
- bit 3: specify whether to change motor spin direction;
- bit 4: specify encoder count direction;
- bit 5: specify whether to change encoder count direction;
- from bit 6 to bit 8: unused.

Note: the encoder value is in little-endian two's complement notation.  

<br>

### ***PID***

Payload of 24 bytes -> message of 25 bytes.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 10010 (**PID**).<br>NUM: selected motor index in [0, N-1].                        |
|   2-5   | Float value for PID encoder error divider.                                    |
|   6-9   | Float value for PID proportional coefficient.                                 |
|  10-13  | Float value for PID integral coefficient.                                     |
|  14-17  | Float value for PID derivative coefficient.                                   |
|  18-21  | Float value for PID integral saturation.                                      |
|  20-25  | Float value for PID dirty derivative pole.                                    |

Note: PID values are expressed in continuos time domain (discretization made on-board according to microcontroller time sampling).

<br>

### ***ACKC***

Payload of N+2 bytes -> message of N+3 bytes.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 11000 (**ACKC**).<br>NUM: number of motor minus one (N-1).                        |
|    2    | Bit-mask for motors end-stop state [87654321].<br> Bit value: 0 = false, 1 = true.      |
|    3    | Bit-mask for values' sign [87654321].<br> Bit value: 0 = positive, 1 = negative.        |
|    4    | Unsigned delta-encoder value in [0, 255] for motor 1.                                   |
|   ...   | ...                                                                                     |
|   N+3   | Unsigned delta-encoder value in [0, 255] for motor N.                                   |

<br>

### ***ACKS***

Payload of 0 bytes -> message of 1 bytes.

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 11001 (**ACKS**).<br>NUM: number of motors (N-1) or selected motor index [0, N-1].|

<br>

### ***ERROR***

| Byte n. | Description                                                                             |
| ------- | --------------------------------------------------------------------------------------- |
| 1 (hdr) | CODE: 11111 (**ERROR**).<br>NUM: number of motors minus one (N-1).                      |

<br><br>

## **Little-endian transmission**

Value composed by 2+ bytes are expressed in little-endian notation, which mean from least to most significant byte. 
For example, for float values (4 bytes) that means:
~~~
float 32 bits:  D8 D7 D6 D5 D4 D3 D2 D1 | C8 C7 C6 C5 C4 C3 C2 C1 | B8 B7 B6 B5 B4 B3 B2 B1 | A8 A7 A6 A5 A4 A3 A2 A1

1st byte sent:  A8 A7 A6 A5 A4 A3 A2 A1
2nd byte sent:  B8 B7 B6 B5 B4 B3 B2 B1
3rd byte sent:  C8 C7 C6 C5 C4 C3 C2 C1
4th byte sent:  D8 D7 D6 D5 D4 D3 D2 D1

bytes sent:     A8 A7 A6 A5 A4 A3 A2 A1 | B8 B7 B6 B5 B4 B3 B2 B1 | C8 C7 C6 C5 C4 C3 C2 C1 | D8 D7 D6 D5 D4 D3 D2 D1
~~~
Note: usually computers store all their data in little-endian format. Rule of thumb:
- Big-endian: human-friendly format.
- Little-endian: computer-friendly format.