### CH341A USB Bridge Chip Interface Documentation

#### CH341_CMD_PARA_INIT (0xB1)
**USB Endpoint:** BULK_OUT  
**Description:** Initializes parallel port mode (EPP or MEM).  
**Parameters:**  
| Byte | Value | Description        |
|------|-------|--------------------|
| 0    | 0xB1  | Command code       |
| 1    | 0x0?  | Mode configuration:<br>- 0x00 EPP Data Mode<br>- 0x01 EPP Address Mode<br>- 0x02 MEM Mode |
| 2    | 0x00  | Reserved           |

---

#### CH341_CMD_GET_STATUS (0xA0)
**USB Endpoint:** BULK_OUT (1 byte command) → BULK_IN (3-byte response)  
**Description:** Reads parallel port status, GPIO states, and interface flags. 
**Response Structure:**  
| Byte | Bit Position | Name       | Type     | Description                          |
|------|--------------|------------|----------|--------------------------------------|
| 0    | [0-7]        | D[7:0]     | GPIO     | Current state of data lines 0-7      |
| 1    | 7            | ERR#       | Flag     | Error condition (0=OK, 1=Error)      |
|      | 8            | PEMP       | Flag     | Parallel port empty (0=Busy, 1=Ready)|
|      | 9            | INT#       | Interrupt| GPIO interrupt request (0=No, 1=Yes)  |
|      | 10           | SLCT       | Control  | Chip select status (0=Inactive, 1=Active)|
| 2    | 8            | D[15:8]    | GPIO     | Current state of data lines 15-8     |
|      | 13           | WAIT#      | Control  | Wait state (0=Busy, 1=Idle)         |
|      | 14           | AUTOFD#    | I²C Only | Auto-feed status (I²C slave mode)    |
|      | 14           | DATAS#     | SPI Only | Data strobe (SPI transfer active)    |
|      | 15           | SLCTIN#    | I²C Only | Address strobe (I²C address phase)   |
|      | 15           | ADDRS#     | SPI Only | SPI address/command phase            |
|      | 23           | SDA        | I²C Only | I²C data line state (if connected)   |

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
     - SDA only appears in I²C transactions

**Implementation Details:**  
- All status bits read as active-high logic (1=active)  
- Unused bits [12:11] and [16:15] (SPI mode) remain reserved  
- Interrupt detection requires prior configuration via SET_OUTPUT command  
- SDA state reflects last I²C transaction state  

For example:  
- If **Byte 1 = 0x05**, this means:  
  - ERR#=0 (no error), PEMP=0 (busy), INT#=1 (interrupt pending), SLCT=0 (not selected)  
- If **Byte 2 = 0x2A** in SPI mode, this shows:  
  - D15=0, D14=0, D13=1, D12=0, D11=1, D10=0 (from bit 8)  
  - WAIT#=0 (active), DATAS#=1 (data phase), ADDRS#=0 (command phase)

---

#### CH341_CMD_SET_OUTPUT (0xA1)
**USB Endpoint:** BULK_OUT (11 bytes)  
**Description:** Configures GPIO direction and output states. First byte 0xA1, second byte 0x6A (fixed), third byte controls update masks.  

**Parameters:**  
| Byte | Value       | Field Name       | Description                          |
|------|-------------|------------------|--------------------------------------|
| 0    | 0xA1        | Command code     | Always 0xA1                          |
| 1    | 0x6A        | Fixed value      | Required constant                    |
| 2    | Mask        | Output Enable    | Selects GPIO banks for update:<br>**Bit 4**: GPIO [23:16]<br>**Bit 5**: GPIO [15:8]<br>**Bit 6**: GPIO [7:0]<br>(1=enable update, 0=no change) |
| 3    | Data Byte 1 | GPIO Data [15:8] | Output values for GPIO 15-8 (LSB-first)|
| 4    | Dir Byte 1  | GPIO Dir [15:8]  | Direction control:<br>**Bit [4:0]**: GPIO [15:11] direction (0=input, 1=output)<br>Bits [7:5] always 0x02 mask |
| 5    | Data Byte 2 | GPIO Data [7:0]  | Output values for GPIO 7-0          |
| 6    | Dir Byte 2  | GPIO Dir [7:0]  | Direction control:<br>**Bit [4:0]**: GPIO [7:3] direction<br>Bits [7:5] always 0x02 mask |
| 7    | Data Byte 3 | GPIO Data [23:16]| Output values for GPIO 23-16        |
| 8-10 | 0x00        | Padding          | Unused                               |

**Output-Only Pins:**  
| Bit | GPIO | Signal | Description          |
|-----|------|--------|----------------------|
| 16  | 16   | RESET# | Active-low reset     |
| 17  | 17   | WRITE# | Active-low write strobe |
| 18  | 18   | SCL    | I²C clock line       |
| 29  | 29   | SDA    | I²C data line        |

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
- GPIO [23:16] can't be configured as inputs  
- GPIO [19:16] direction controlled by Mask Byte 2, Bit 4 but lack dedicated data bytes  
- SDA (bit 29) and SCL (bit 18) only function in I²C mode

---

#### CH341_CMD_I2C_STREAM (0xA1)
**USB Endpoint:** BULK_OUT  
**Description:** I²C protocol commands  
**Parameters:**  
- 0xA1 0xUS [delay]: Adds microsecond delay (LSB-first)  
- 0xA1 0xSS: SPI stream configuration (S=speed bits)  
- 0xA1 0xSE: SPI stream end  

**SPI Chip Select:**  
Use UIO_STM_OUT (0xAB 0x80) with masks:  
- 0x36 → CS inactive (all outputs high)  
- 0x35 → CS via D1  
- 0x33 → CS via D2  
- 0x27 → CS via D4

---

#### CH341_CMD_SPI_STREAM (0xA8)
**USB Endpoint:** BULK_OUT (command+data) → BULK_IN (response)  
**Description:** Performs SPI transfer. First byte is command (0xA8), followed by data payload (up to 32 bytes).  
**Parameters:**  
- Byte 0: 0xA8 (Command code)  
- Byte 1-N: SPI data payload (LSB-first per byte)  

**Response:** Byte count matches data payload length

---

#### CH341_CMD_UIO_STREAM (0xAB)
**USB Endpoint:** BULK_OUT (command+data) → BULK_IN (optional response)  
**Description:** Multi-function command for GPIO manipulation, SPI transfers, and I²C transactions. Uses 3-byte subcommand structure.  

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

##### SPI_STM_CS (0x3?)
**Description:** SPI Chip Select Control  
| Byte | Value | Field       | Description               |
|------|-------|-------------|---------------------------|
| 0    | 0xAB  | Stream Code | Always 0xAB               |
| 1    | 0x3?  | Subcommand  | Chip select control:<br>- 0x30: All CS inactive<br>- 0x31: CS via D1<br>- 0x32: CS via D2<br>- 0x33: CS via D4<br>*Only bits [1:0] determine chip select* |
| 2    | 0x??  | CS Pattern  | Unused (driver sets CS via subcommand) |

**No Response**

##### SPI_STM_DATA (0xA8)
**Description:** SPI Transfer  
| Byte | Value | Field        | Description          |
|------|-------|--------------|----------------------|
| 0    | 0xAB  | Stream Code  | Always 0xAB          |
| 1    | 0xA8  | Subcommand   | SPI data transfer    |
| 2    | 0x??  | Data Byte 1  | First SPI data byte  |
| ...  | ...   | ...          | Up to 32 bytes       |

**Response:**  
| Byte | Bit Range | Field   | Description      |
|------|-----------|---------|------------------|
| 0    | [7:0]     | Data Byte 1 | MISO data (if connected) |
| ...  | ...       | ...     | Matches TX length |

##### I2C_STM_DELAY (0xA1)
**Description:** Add I²C microsecond delay  
*requires 2nd 0xAB 0xA1 command for MSB*    
| Byte | Value | Field       | Description        |
|------|-------|-------------|--------------------|
| 0    | 0xAB  | Stream Code | Always 0xAB        |
| 1    | 0xA1  | Subcommand  | I²C delay start    |
| 2    | 0x??  | Delay LSB   | 8 of 16-bit delay value (LSB-first) |

**No Response**  

**Notes:**  
- **I²C Timing:** Delay value = (Byte 2 of second 0xA1 command × 256) + Byte 2 of first 0xA1 command  
  `0xAB 0xA1 [Delay_LSB] → 0xAB 0xA1 [Delay_MSB]`  
  Total delay = (MSB×256 + LSB) μs  

##### I2C_STM_READ (0xB?)
**Description:** I²C Read N bytes (requires prior address write)  
| Byte | Value | Field      | Description          |
|------|-------|------------|----------------------|
| 0    | 0xAB  | Stream Code| Always 0xAB          |
| 1    | 0xB?  | Subcommand | 0xB0 + N (bytes to read) |
| 2    | 0x??  | Unused     | Must be 0x00        |

**Response:**  
| Byte | Bit Range | Field    | Description         |
|------|-----------|----------|---------------------|
| 0    | [7:0]     | Data Byte 1 | First I²C response byte |
| ...  | ...       | ...      | N total bytes       |

##### I2C_STM_STOP (0xA2)
**Description:** I²C Stop Condition  
| Byte | Value | Field        | Description      |
|------|-------|--------------|------------------|
| 0    | 0xAB  | Stream Code  | Always 0xAB      |
| 1    | 0xA2  | Subcommand   | I²C stop          |
| 2    | 0x00  | Unused      | Must be 0x00     |

**No Response**

#### **Implementation Notes:**  
- SPI transfers require 0xAB 0xA8 [Data...] sequence  
- I²C uses 0xAB 0xA1/0xB?/0xA2 for protocol control  
- GPIO stream ends with `0xAB 0x20 [Unused]`  
- All data uses LSB-first byte packing  

**Example Subcommand Sequence:**
**SPI Write 4 Bytes:**  
`[0xAB 0xA8 0x11] [0xAB 0xA8 0x22] [0xAB 0xA8 0x33] [0xAB 0xA8 0x44] [0xAB 0x20 0x00]`  
Sends 0x11, 0x22, 0x33, 0x44 via SPI  

**I²C Read 2 Bytes:**  
`[0xAB 0xA1 0x05] [0xAB 0xA1 0x00] [0xAB 0xB2 0x00] → [Response Data: 0xAA 0xBB]`  
Reads 2 bytes with 5μs delay  

---

#### Interrupt Handling
**INT# Pin:**  
- Active-low hardware interrupt  
- Requires driver configuration through `CH34x_START_IRQ_TASK` ioctl  

**GPIO Interrupts:**  
- Enabled via 0xA1 command's direction mask  
- Bit 5 (interrupt enable) and bit 3 (trigger status) in interrupt buffer  

---

### Notes
- All multi-byte values use little-endian format  
- MEM mode uses 32-byte bulk transfers  
- SPI supports 1-32 byte transfers with optional CS control  
- I²C requires 3-byte command sequences for transactions  
- Output-only pins: RESET# (16), WRITE# (17), SCL (18), SDA (29)  

**Status Flags:**  
- BUSY/WAIT# indicates active transfer  
- DATAS#/AUTOFD# polarity depends on interface configuration  
- ERR# shows hardware fault condition  

**Driver Support:**  
- Verified against driver v1.5 (2024.12)  
- Supports CH341/CH347/CH339/CH346 variants  
- Requires 16-byte alignment for bulk transfers in EPP/MEM modes  

**Maximum Values:**  
- EPP/MEM transfers: 31 bytes  
- SPI transfers: 32 bytes  
- I²C addresses: 2-byte (EEPROM >256 bytes)  
- Interrupt detection: 8 GPIO channels
