### CH341A USB Bridge Chip Interface Documentation

- All commands support both **USB Control OUT** and **USB Bulk OUT** endpoints unless explicitly stated otherwise
- All multi-byte values use little-endian format
- All delays require explicit values (no default timing)  

**Maximum Values:**  
- EPP/MEM transfers: 31 bytes  
- SPI transfers: 32 bytes  
- I²C addresses: 2-byte (EEPROM >256 bytes)  
- Interrupt detection: 8 GPIO channels  

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
| Byte | Value       | Description               |
|------|-------------|---------------------------|
| 0    | 0xB1 | Command code |
| 1    | 0x00/0x02 | Mode low byte:<br>- 0x00: EPP Mode<br>- 0x02: MEM Mode |
| 2    | 0x00 | Mode high byte |

---

#### CMD_GET_STATUS (0xA0)
**USB Endpoint:** BULK_OUT (1 byte command) → BULK_IN (3-byte response)  
**Description:** Reads parallel port status, GPIO states, and interface flags.  

**Response Structure:**  
| Byte | Bit Pos | Name   | Type   | Description                  |
|------|---------|--------|--------|------------------------------|
| 0    | [0-7]   | D[7:0] | GPIO   | Current state of data lines 0-7 |
| 1    | 7       | ERR#   | Flag   | Error condition (0=OK, 1=Error) |
|      | 8       | PEMP   | Flag   | Parallel port empty (0=Busy, 1=Ready) |
|      | 9       | INT#   | Flag   | Interrupt pending (0=No,1=Yes)  |
|      | 10      | SLCT   | Control | Chip select active (0=Inactive, 1=Active)   |
| 2    | 8       | D[15:8]| GPIO   | Current state of data lines 15-8 |
|      | 13      | WAIT#  | Control | Wait state (0=Busy, 1=Idle)          |
|      | 14      | AUTOFD#    | I²C Only | Auto-feed status (I²C slave mode)    |
|      |         | DATAS#     | SPI Only | Data strobe (SPI transfer active)    |
|      | 15      | SLCTIN#    | I²C Only | Address strobe (I²C address phase)   |
|      |         | ADDRS#     | SPI Only | SPI address/command phase            |

**Status Flags:**  
- BUSY/WAIT# indicates active transfer  
- DATAS#/AUTOFD# polarity depends on interface configuration  
- ERR# shows hardware fault condition  

**Clarifications:**  
- **Bit Position** shows exact location within response byte (0 = least significant)  
- **Name** uses standard hardware signal notation  
- **Type** indicates signal category:  
  - GPIO: Direct pin state read  
  - Flag: Status indicator  
  - Control: Bus control signal state  
  - I²C Only: Only valid in I²C interface mode  
  - SPI Only: Only valid in SPI interface mode  

**Hardware Behavior:**  
1. **Byte 0** contains raw GPIO values (D0=LSB, D7=MSB)  
2. **Byte 1** contains critical status flags:  
   - ERR# = 1 when hardware fault detected  
   - PEMP = 1 when parallel port ready for next operation  
   - INT# = 1 when GPIO interrupt pending  
   - SLCT shows active chip selection  
3. **Byte 2** contains:  
   - Extended GPIO state (D15-D8) in bit 8  
   - Mode-specific control signals:  
     - WAIT# shows bus idle state  
     - AUTOFD#/DATAS# polarity depends on interface mode  
     - SLCTIN#/ADDRS# indicates protocol phase  
     - ADDRS#/SLCTIN# polarity depends on interface

**Implementation Details:**  
- All status bits read as active-high logic (1=active)  
- Unused bits [12:11] and [16:15] (SPI mode) remain reserved  
- Interrupt detection requires prior configuration via SET_OUTPUT command  

For example:  
- If **Byte 1 = 0x05**, this means:  
  - ERR#=0 (no error), PEMP=0 (busy), INT#=1 (interrupt pending), SLCT=0 (not selected)  
- If **Byte 2 = 0x2A** in SPI mode, this shows:  
  - D15=0, D14=0, D13=1, D12=0, D11=1, D10=0 (from bit 8)  
  - WAIT#=0 (active), DATAS#=1 (data phase), ADDRS#=0 (command phase)

---

#### CMD_SET_OUTPUT (0xA1)
**USB Endpoint:** BULK_OUT (11 bytes)  
**Description:** Configures GPIO direction and output states. First byte 0xA1, second byte 0x6A (fixed), third byte controls update masks.  

**Parameters:**  
| Byte | Value       | Field Name       | Description                          |
|------|-------------|------------------|--------------------------------------|
| 0    | 0xA1        | Command code     | Always 0xA1                          |
| 1    | 0x6A        | Fixed value      | Required constant                    |
| 2    | Mask        | Output Enable    | Selects GPIO banks for update:<br>**Bit 4**: GPIO [23:16] (controls RESET#, WRITE#, SCL, SDA)<br>**Bit 5**: GPIO [15:8]<br>**Bit 6**: GPIO [7:0]<br>(1=enable update, 0=no change) |
| 3    | Data Byte 1 | GPIO Data [15:8] | Output values for GPIO 15-8 (LSB-first)|
| 4    | Dir Byte 1  | GPIO Dir [15:8]  | Direction control:<br>**Bit [4:0]**: GPIO [15:11] direction (0=input, 1=output)<br>Bits [7:5] always 0x02 mask |
| 5    | Data Byte 2 | GPIO Data [7:0]  | Output values for GPIO 7-0          |
| 6    | Dir Byte 2  | GPIO Dir [7:0]  | Direction control:<br>**Bit [4:0]**: GPIO [7:3] direction<br>Bits [7:5] always 0x02 mask |
| 7    | Data Byte 3 | GPIO Data [23:16]| Output values for GPIO 23-16        |
| 8-10 | 0x00        | Padding          | Unused                               |

**Output-Only Pins:**  
| GPIO | Signal | Description          |
|------|--------|----------------------|
| 16   | RESET# | Active-low reset     |
| 17   | WRITE# | Active-low write strobe |
| 18   | SCL    | I²C clock line       |
| 19   | SDA    | I²C data line        |

**Notes:**  
1. Mask byte (Byte 2) controls **which GPIO banks** get updated, not individual bits:  
   - Bit 4 → Updates RESET#, WRITE#, SCL, SDA (always output)  
   - Bit 5 → Updates GPIO [15:8] (direction + data)  
   - Bit 6 → Updates GPIO [7:0] (direction + data)  

2. Direction bytes use **internal 0x10 offset**:  
   - For GPIO [15:8]: Only bits [4:0] of Dir Byte 1 affect pins [15:11]  
   - For GPIO [7:0]: Only bits [4:0] of Dir Byte 2 affect pins [7:3]  
   - Remaining bits always write 0x02 (matches driver implementation)  

**Bit-to-Pin Mapping Consistency:**  
- GET_STATUS returns GPIO [15:8] in **Byte 2, Bit 8**  
- SET_OUTPUT writes GPIO [23:16] in **Byte 7** and controls their direction via **Mask Byte 2, Bit 4**  
- GPIO [23:16] always operate as outputs but their state comes from:  
  - Byte 7 (Data) for RESET#, WRITE#, etc.  
  - Byte 3/4 (Data/Dir) for GPIO 19-16 (if enabled)  

**Example Configuration:**  
To set GPIO [7:0] as outputs and GPIO [23:16] to fixed values:  
- Byte 2 = 0x30 (enable GPIO [7:0] + [23:16])  
- Byte 3 = 0x00 (no change to GPIO [15:8])  
- Byte 4 = 0x1F (set GPIO [15:11] as outputs - unused in practice)  
- Byte 5 = 0xFF (set GPIO [7:0] to high)  
- Byte 6 = 0x1F (set GPIO [7:3] as outputs)  
- Byte 7 = 0x0F (set RESET#=0, WRITE#=1, SCL=0, SDA=1)  

**Critical Notes:**  
- GPIO [23:16] always operate as outputs but state comes from:  
  - Byte 7 (Data) for RESET#/WRITE#/SCL/SDA  
  - Byte 3/4 (Data/Dir) for GPIO [19:16] when enabled via Mask Byte 2, Bit 4  
- GPIO [19:16] direction controlled by Mask Byte 2, Bit 4 but lack dedicated data bytes  
- SDA (bit 19) and SCL (bit 18) only function in I²C mode

---

#### CMD_SPI_STREAM (0xA8)  
**USB Endpoint:** BULK_OUT (command+data) → BULK_IN (response)  
**Description:** SPI data transfer
**Parameters:**  
| Byte | Value       | Field        | Description          |
|------|-------------|--------------|----------------------|
| 0    | 0xA8        | Command      | SPI transfer start   |
| 1-N  | Data Bytes  | SPI Payload  | Up to 32 bytes LSB-first|

**Response:**  
| Byte | Bit Range | Field   | Description      |
|------|-----------|---------|------------------|
| 0-N  | [7:0]     | Data Bytes | MISO data (if connected) |

**Chip Select Control:**  
Use `CMD_UIO_STREAM UIO_STM_OUT (0xAB 0x80)` with masks:  
| Mask | CS Behavior       |
|------|-------------------|
| 0x36 | All CS inactive   |
| 0x35 | CS via GPIO 1     |
| 0x33 | CS via GPIO 2     |
| 0x27 | CS via GPIO 4     |

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
**Description:** Unified interface for GPIO and SPI operations  

**Subcommands:**  

##### UIO_STM_IN (0x00)
**Description:**  Read GPIO [7:0]  
| Byte | Value | Field        | Description                   |
|------|-------|--------------|-------------------------------|
| 0    | 0xAB  | Stream Code  | Always 0xAB for subcommands   |
| 1    | 0x00  | Subcommand   | GPIO input read               |
| 2    | 0x??  | Read Mask    | Unused by hardware (driver filter only) |

**Response:**  
| Byte | Bit Range | Field  | Description      |
|------|-----------|--------|------------------|
| 0    | [7:0]     | GPIO[7:0] | Current pin states |

##### UIO_STM_DIR (0x40)
**Description:** Sets GPIO direction.  
| Byte | Value | Field       | Description                      |
|------|-------|-------------|----------------------------------|
| 0    | 0xAB  | Stream Code | Always 0xAB                      |
| 1    | 0x40  | Subcommand  | GPIO direction control           |
| 2    | 0x??  | Dir Mask    | Mask Structure:<br>- Bits [5:0]: GPIO [5:0] direction (0=input, 1=output)<br>- Bits [7:6] always 0 |

**No Response**  

##### UIO_STM_OUT (0x80)
**Description:** Write GPIO [5:0]  
| Byte | Value | Field      | Description                  |
|------|-------|------------|------------------------------|
| 0    | 0xAB  | Stream Code| Always 0xAB                  |
| 1    | 0x80  | Subcommand | GPIO output write            |
| 2    | 0x??  | Data       | Data Structure:<br>- Bits [5:0]: GPIO [5:0] output state<br>- Bits [7:6] always 0 |

**No Response**

##### UIO_STM_END (0x20)
**Description:** Terminate Stream  
| Byte | Value | Field        | Description          |
|------|-------|--------------|----------------------|
| 0    | 0xAB  | Stream Code  | Always 0xAB          |
| 1    | 0x20  | Subcommand   | End sequence         |
| 2    | 0x00  | Unused      | Must be 0x00         |

**No Response**

---

#### Interrupt Handling
 
Supports hardware interrupts on one configurable GPIO pin. Key configuration parameters:

**Hardware Interrupt Pin**  
- Only GPIO 19 (D4) supports hardware interrupts  
- Must be configured as input in board configuration  
- Requires external pull-up resistor  

**Interrupt Polarity**  
- Active-low interrupt signal (INT#)  
- Triggers on rising edges (0→1 transitions after debouncing)  

**Configuration Requirements**  
- Enable interrupt detection via `CH341_CMD_SET_OUTPUT` or `CH341_CMD_UIO_STREAM UIO_STM_OUT` command's direction mask  
- Configure target GPIO as input (Bit 5=1 in direction mask)  
- Use interrupt buffer status bits:  
  - Bit 5: Interrupt enable (1=active)  
  - Bit 3: Trigger status (1=pending interrupt)  

**Hardware Limitations**  
- Only one GPIO can generate hardware interrupts  
- Interrupt detection requires USB polling for status updates  
- Interrupts share bandwidth with normal USB transfers  

**Software interrupts**
- Software polling for GPIOs 0-7 and 15-18 can be implemented if needed.

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
| UIO_STM_US | 0xC0 | UIO Interface Delay Command (us) |
