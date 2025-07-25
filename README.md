### CH341A USB Bridge Chip Interface Documentation

**Maximum Values:**  
- EPP/MEM transfers: 31 bytes  
- SPI transfers: 32 bytes  
- I²C addresses: 2-byte (EEPROM >256 bytes)  
- Interrupt detection: 8 GPIO channels  (CH347 only) 

---

#### VENDOR_VERSION (0x5F)  
**USB Endpoint:** USB Control IN  
**Description:** Returns chip version identifier  

**Response:**  
| Byte | Value | Description         |
|------|-------|---------------------|
| 0    | 0x??  | Version low byte    |
| 1    | 0x??  | Version high byte   |

**Common Response:**  
- 0x0030 indicates CH341A revision

---

#### CMD_PARA_INIT (0xB1)  
**USB Endpoint:** USB Control OUT  
**Description:** Initializes parallel port mode (EPP or MEM).  
**Parameters:**  
| Byte | Value          | Description  |
|------|----------------|--------------|
| 0    | 0xB1           | Command code |
| 1    | 0x00/0x01/0x02 | Mode:<br>- 0x00/0x01: EPP Mode<br>- 0x02: MEM Mode |
| 2    | 0x02           | Fixed value  |

---

#### CMD_GET_STATUS (0xA0)
**USB Endpoint:** BULK_OUT (1 byte command) → BULK_IN (6-byte response)  
**Description:** Reads parallel port status, GPIO states, and interface flags.  

**Response Structure:**  
| Byte | Bit | Pin | Description     |
|------|-----|-----|-----------------|
| 0    | 0   | D0  | CTS#, D0, CS0   |
|      | 1   | D1  | DSR#, D1, CS1   |
|      | 2   | D2  | RI#,  D2, CS2   |
|      | 3   | D3  | DCD#, D3, DCK   |
|      | 4   | D4  | OUT#, D4, DOUT2 |
|      | 5   | D5  | DTR#, D5, DOUT  |
|      | 6   | D6  | RTS#, D6, DIN2  |
|      | 7   | D7  | SLP#, D7, DIN   |
| 1    | 0   | D8  | TXD,  ERR#      |
|      | 1   | D9  | RXD,  PEMP      |
|      | 2   | D10 | INT#, ACK#      |
|      | 3   | D11 | IN3,  SLCT      |
|      | 4   | -   | Reserved        |
|      | 5   | D13 | TEN#, BUSY, WAIT# |
|      | 6   | D14 | ROV#, AFD#, DS# |
|      | 7   | D15 | IN7,  SIN#, AS# |
| 2    | 0   | D16 | TNOW, IN#, RST  |
|      | 1   | D17 | RDY#, STB#, WR# |
|      | 2   | D18 | SCL             |
|      | 3   | D19 | SDA             |
|      | [7:4] | - | Reserved        |
| 3    | [7:0] | - | Reserved        |
| 4    | [7:0] | - | Reserved        |
| 5    | [7:0] | - | Reserved        |

**Notes:**  
- Status: 0=low, 1=high
- Available for both input and output direction pins
- Interrupt detection requires prior configuration via SET_OUTPUT command  

---

#### CMD_SET_OUTPUT (0xA1)
**USB Endpoint:** BULK_OUT (11 bytes)  
**Description:** Configures GPIO direction and output states.

**Parameters:**  
| Byte | Value       | Field Name       | Description                          |
|------|-------------|------------------|-----------------|
| 0    | 0xA1        | Command code     | Always 0xA1     |
| 1    | 0x6A        | Fixed value      | Always 0x6A     |
| 2    | Mask        | Output Enable    | Selects GPIO banks for update:<br>- **bits [1:0]**: fixed to 0b11<br>- **bit 2**: 0b1 to update bytes 3-4<br>- **bit 3**: 0b1 to update bytes 5-6<br>- **bit 4**: 0b1 to set byte 7<br>- **bits [7:5]**: fixed to 0b000<br>(1=enable update, 0=no change) |
| 3    | Data Byte 1 | GPIO Data [15:8] | Output values for D15-D8: 0=low, 1=high |
| 4    | Dir Byte 1  | GPIO Dir [15:8]  | Direction control for D15-D8: 0=input, 1=output |
| 5    | Data Byte 2 | GPIO Data [7:0]  | Output values for D7-D0: 0=low, 1=high |
| 6    | Dir Byte 2  | GPIO Dir [7:0]   | Direction control for D7-D0: 0=input, 1=output |
| 7    | Data Byte 3 | GPIO Data [19:16] | **bits [3:0]**: Output values for D19-D16 (output only pins)<br>**bits [7:4]**: padding |
| 8-10 | 0x00        | Padding         | Unused   |

**Output-Only Pins:**  
| GPIO | Signal | Description          |
|------|--------|----------------------|
| 16   | RESET# | Active-low reset     |
| 17   | WRITE# | Active-low write strobe |
| 18   | SCL    | Open-drain I²C clock line |
| 19   | SDA    | Open-drain I²C data line |

**Notes:**  
1. Mask byte (Byte 2) controls **which GPIO banks** get updated, not individual bits:
   - Bit 2 → Updates GPIO [15:8] (direction + data)
   - Bit 3 → Updates GPIO [7:0] (direction + data)  
   - Bit 4 → Updates RESET#, WRITE#, SCL, SDA (always output)  

---

#### CMD_SPI_STREAM (0xA8)

**USB Endpoint:** BULK_OUT (command+data) → BULK_IN (response)  
**Description:** Performs SPI data transfer using the currently configured GPIO[5:0] as SPI lines. The actual SPI transfer behavior (chip select, clock polarity, I/O direction, etc.) is determined by the state of these GPIO pins, which must be explicitly configured using `CMD_UIO_STREAM` subcommands prior to issuing the SPI stream command.

**Parameters:**  
| Byte | Value       | Field        | Description                       |
|------|-------------|--------------|-----------------------------------|
| 0    | 0xA8        | Command      | SPI transfer start                |
| 1-N  | Data Bytes  | SPI Write Bytes | MOSI data to write to DOUT/DOUT2. Up to 32 bytes, LSB-first. |

**Response:**  
*Only supports full-duplex streaming. The number of bytes read always matches the number of written bytes.*  
| Byte | Bit Range | Field         | Description                   |
|------|-----------|---------------|-------------------------------|
| 0-N  | [7:0]     | SPI Read Bytes | Received MISO data on DIN/DIN2. LSB-first. |

**SPI Line and Mode Configuration:**  
SPI bus lines and protocol options are actively controlled via the state and direction of GPIO[0:5], which are set using `CMD_UIO_STREAM` with the following subcommands:

- **UIO_STM_DIR (0x40 | mask):** Set direction for GPIO[5:0].  
    - Bit = 1: output  
    - Bit = 0: input  
- **UIO_STM_OUT (0x80 | value):** Set output level for GPIO[5:0].  
    - Bit = 1: high  
    - Bit = 0: low  
- **UIO_STM_END (0x20):** Marks the end of the subcommand stream.

**SPI Pins Mapping:**  
| GPIO | Signal | Description                |
|------|--------|----------------------------|
| 0    | CS0    | Chip Select 0 (active low) |
| 1    | CS1    | Chip Select 1 (active low) |
| 2    | CS2    | Chip Select 2 (active low) |
| 3    | DCK    | SPI Clock                  |
| 4    | DOUT2  | Dual Output (not always used) |
| 5    | DOUT   | MOSI (output)              |

**CPOL (Clock Polarity) and 3-wire (Half-duplex) Support:**  
- **CPOL:** Set DCK (GPIO3) output level for idle state (low for CPOL=0, high for CPOL=1) before issuing the SPI stream.  
- **3-wire (Half-duplex):** Change DOUT (GPIO5) direction between input and output using UIO_STM_DIR. For RX, set DOUT as input; for TX, set as output. *Note: DIN pin is always used as MISO to read data, no matter if DOUT is set to input mode.*  
- The state and direction of each pin **at the time the SPI stream is issued** determines the SPI protocol behavior.  

**Chip Select Handling:**  
- Chip select lines (CS0/CS1/CS2) must be asserted/deasserted by setting GPIO0/1/2 output state appropriately using UIO_STM_OUT before and after the SPI transfer.  

**Example:**
```c
// 1. Assert CS0 (set CS0 low), set DCK low (CPOL=0), set DOUT as output
[0xAB, 0x40 | dir_mask, 0x80 | out_mask, 0x20]

// 2. Send SPI data
[0xA8, ...data...]

// 3. Deassert CS0 (set CS0 high)
[0xAB, 0x40 | dir_mask, 0x80 | out_mask, 0x20]
```

**Notes:**  
- **CMD_SPI_STREAM only transfers data.** All SPI bus configuration (chip select, clock polarity, direction, 3-wire mode) must be set via UIO_STM_DIR and UIO_STM_OUT before each transfer phase.  
- **The output and direction settings of GPIO[0:5] are not changed by SPI stream commands.** They must be managed explicitly by the host.
- Data bytes are transferred using lowest significant bit (LSB) first format.  

---

#### CMD_I2C_STREAM (0xAA)
**USB Endpoint:** BULK_OUT (command+data) → BULK_IN (optional response)    
**Description:** I²C protocol command stream  

**Subcommands:**

##### I2C_STM_US (0x40)
**Description:** Sets I²C delay in microseconds  
| Byte | Value | Description                 |
|------|-------|-----------------------------|
| 0    | 0xAA  | Stream code                 |
| 1    | 0x40  | Microsecond delay subcommand|
| 2    | Delay | Delay value (1-15 µs)       |

**No Response**  

##### I2C_STM_MS (0x50)
**Description:** Sets I²C delay in milliseconds  
| Byte | Value | Description                 |
|------|-------|-----------------------------|
| 0    | 0xAA  | Stream code                 |
| 1    | 0x50  | Millisecond delay subcommand|
| 2    | Delay | Delay value (1-15 ms)       |

**No Response**  

##### I2C_STM_SET (0x60)
**Description:** Sets I²C bus speed configuration  
| Byte | Value     | Field        | Description          |
|------|-----------|--------------|----------------------|
| 0    | 0xAA      | Stream Code  | Always 0xAA        |
| 1    | 0x60      | Subcommand   |  |
| 2    | 0x00-0x03 | I²C Speed    | **I²C clock rate:**<br>- 0x00: 20 kHz<br>- 0x01: 100 kHz<br>- 0x02: 400 kHz<br>- 0x03: 750 kHz |
|      | Bit 7     | SPI Mode     | **SPI Mode:**<br>- 0: SPI single io<br>- 1: SPI dual io|

**No Response**

##### I2C_STM_STA (0x74)
**Description:** Generates I²C start condition  
| Byte | Value       | Field        | Description          |
|------|-------------|--------------|----------------------|
| 0    | 0xAA        | Stream Code  | Always 0xAA          |
| 1    | 0x74        | Subcommand   | Start condition      |
| 2    | Unused      | -            | Must be 0x00         |

**No Response**

##### I2C_STM_STO (0x75)
**Description:** Generates I²C stop condition  
| Byte | Value       | Field        | Description          |
|------|-------------|--------------|----------------------|
| 0    | 0xAA        | Stream Code  | Always 0xAA          |
| 1    | 0x75        | Subcommand   | Stop condition       |
| 2    | Unused      | -            | Must be 0x00         |

**No Response**

##### I2C_STM_OUT (0x80)
**Description:** Writes I²C data bytes  
| Byte | Value       | Field        | Description          |
|------|-------------|--------------|----------------------|
| 0    | 0xAA        | Stream Code  | Always 0xAA          |
| 1    | 0x80        | Subcommand   | Data write           |
| 2    | Len Byte    | Byte Count   | Number of bytes to write (N) |
| 3-N  | Data Bytes  | Payload      | Up to 31 bytes       |

**No Response**

##### I2C_STM_IN (0xC0)
**Description:** Reads I²C data bytes  
| Byte | Value       | Field        | Description          |
|------|-------------|--------------|----------------------|
| 0    | 0xAA        | Stream Code  | Always 0xAA          |
| 1    | 0xC0        | Subcommand   | Data read            |
| 2    | Len Byte    | Byte Count   | Number of bytes to read (N) |

**Response:** via BULK_IN (N bytes)

##### I2C_STM_END (0x00)  
**Description:** Terminates I²C command stream sequence  
**Parameters:**  
| Byte | Value       | Field        | Description          |
|------|-------------|--------------|----------------------|
| 0    | 0xAA        | Stream Code  | Always 0xAA          |
| 1    | 0x00        | Subcommand   | End sequence         |
| 2    | Unused      | -            | Must be 0x00         |

**Behavior:**  
- Required as final entry in all I²C command sequences  
- Marks end of transaction buffer  
- No response generated  
- Must follow SET/STA/STO/OUT/IN subcommands  

**I²C Address Handling**  
The driver must implements standard 7-bit I²C addressing with an 8-bit format where:  
- The **higher 7 bits** (1-7) carry the actual I²C address  
- **Bit 0** (MSB) serves as the read/write flag:  
  - `0` = Write operation  
  - `1` = Read operation  

For example, address `0x50` becomes `0xA0` (0x50 << 1) during write operations and `0xA1` (0x50 << 1 | 1) during read operations. This left-shift by 1 and R/W flag addition follows conventional I²C protocol requirements. Always verify address formatting matches target device expectations.

**Example Usage:**  
To write 3 bytes to address 0x50:  
```
[0xAA, 0x74, 0x00]  // START
[0xAA, 0x80, 0x04, 0xA0, 0x01, 0x02, 0x03] // WRITE 3 bytes
[0xAA, 0x75, 0x00]  // STOP
[0xAA, 0x00, 0x00]  // END
```

**Critical Notes:**  
- Failure to include END subcommand will leave transaction incomplete  
- All I²C operations require proper START/END framing  
- Must be preceded by valid I²C_STREAM subcommand (never first entry)  

The END subcommand serves as a required terminator for all I²C operations. Its omission would prevent proper transaction closure and must be included in all command sequences following the pattern:  
`[I2C_STM_*] → [I2C_STM_END]`

---

#### CMD_UIO_STREAM (0xAB)
**USB Endpoint:** BULK_OUT (command+data) → BULK_IN (optional response)  
**Description:** Unified interface for direct GPIO and SPI pins control. Accepts a sequence of subcommands, each encoded as a single byte, in a single command stream. This mechanism is used for fine-grained, atomic manipulation of GPIO[0:5] for SPI and general GPIO use.  

**Subcommands:**  

##### UIO_STM_IN (0x00)
**Description:**  Read GPIO [7:0]  
| Byte | Value | Field        | Description          |
|------|-------|--------------|--------------------------------|
| 0    | 0xAB  | Stream Code  | Always 0xAB for subcommands |
| ..   | ..    | ..           | Other subcommands |
| n    | 0x00  | Subcommand   | GPIO input read           |
| ..   | ..    | ..           | Other subcommands |

**Response:**  
| Byte | Bit Range | Field  | Description      |
|------|-----------|--------|------------------|
| n    | [7:0]     | GPIO[7:0] | Current state of D7-D0 lines |

##### UIO_STM_DIR (0x40)
**Description:** Sets direction for GPIO [5:0]  
| Byte | Value | Field       | Description                      |
|------|-------|-------------|----------------------------------|
| 0    | 0xAB  | Stream Code | Always 0xAB                      |
| ..   | ..    | ..           | Other subcommands |
| n    | 0x40 \| dir_mask | dir_mask | Direction control:<br>Bits 0-5: GPIO[5:0] (0=input, 1=output) |
| ..   | ..    | ..           | Other subcommands |

**No Response**  

##### UIO_STM_OUT (0x80)
**Description:** Write output value for GPIO [5:0]  
| Byte | Value | Field      | Description                  |
|------|-------|------------|------------------------------|
| 0    | 0xAB  | Stream Code| Always 0xAB                  |
| ..   | ..    | ..           | Other subcommands |
| n    | 0x80 \| out_mask | out_mask | Output value:<br>Bits 0-5: GPIO[5:0] output data (0=low, 1=high) |
| ..   | ..    | ..           | Other subcommands |

**No Response**

##### UIO_STM_US (0xC0)
**Description:** Insert delay in microseconds  
| Byte | Value | Field      | Description                  |
|------|-------|------------|------------------------------|
| 0    | 0xAB  | Stream Code| Always 0xAB                  |
| ..   | ..    | ..           | Other subcommands |
| n    | 0xC0 \| delay | delay | Delay in microseconds (up to 6 bits) |
| ..   | ..    | ..           | Other subcommands |

**No Response**

##### UIO_STM_END (0x20)
**Description:** Terminate Stream  
| Byte | Value | Field        | Description          |
|------|-------|--------------|----------------------|
| 0    | 0xAB  | Stream Code  | Always 0xAB          |
| ..   | ..    | ..           | Other subcommands |
| N    | 0x20  | Subcommand   | End sequence         |

**No Response**

**Usage Pattern:**
- **Multiple subcommands** can be concatenated in a single CMD_UIO_STREAM. The device will execute them in order.
*Multiple subcommands per stream are allowed and normal. It is not required to send each subcommand in a separate transfer.*
- Each subcommand is a single byte: the upper bits identify the operation, lower bits are the mask/data/delay.
- For full protocol compliance, always finish your sequence with UIO_STM_END.

**Example:**
```c
// Set GPIO[5:0] as outputs, set CS0 low, DCK low, DOUT output, then end
[0xAB, 0x40 | 0x3F, 0x80 | 0x01, 0x20]
```
- This sets all SPI pins as outputs, asserts CS0 (bit 0 low), and ends the stream.

---

#### Interrupt Handling
 
Supports hardware interrupts on one configurable GPIO pin. Key configuration parameters:

**Hardware Interrupt Pin**  
- Only --GPIO 19 (D4)-- supports hardware interrupts  
- Must be configured as input in board configuration  
- Requires external pull-up resistor  

**Interrupt Polarity**  
- Active-low interrupt signal (INT#)  
- Triggers on rising edges (0→1 transitions after debouncing)  

**Configuration Requirements**  
- Enable interrupt detection via `CH341_CMD_SET_OUTPUT` or `CH341_CMD_UIO_STREAM UIO_STM_OUT` command's direction mask  
- Configure target GPIO as input (--Bit 5=1-- in direction mask)  
- Use interrupt buffer status bits:  
  - --Bit 5--: Interrupt enable (1=active)  
  - --Bit 3--: Trigger status (1=pending interrupt)  

**Hardware Limitations**  
- Only one GPIO can generate hardware interrupts  
- Interrupt detection requires USB polling for status updates  
- Interrupts share bandwidth with normal USB transfers  

**Software interrupts**
- Software polling for GPIOs 0-15 can be implemented if needed.

---

#### Undocumented constants
The following constants are defined but unused/undocumented:
| Command | Value | Comments |
|---------|-------|----------|
| I2C_STATUS | 0x52 | get I2C status |
| DEBUG_READ | 0x95 | read two regs |
| DEBUG_WRITE | 0x9A | write two regs to USB Control-OUT 0x2525 when setting parallel mode |
| I2C_CMD_X | 0x54 | send I2C command |
| DELAY_MS | 0x5E | |
| CMD_IO_ADDR | 0xA2 | MEM IO Addr |
| CMD_PRINT_OUT | 0xA3 | print output |
| CMD_SIO_STREAM | 0xA9 | SIO command |
| CMD_BUF_CLEAR | 0xB2 | clear uncompleted data |
