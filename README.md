### CH341 Command Documentation

#### CH341_CMD_PARA_INIT (0xB1)
**USB Endpoint:** BULK_OUT  
**Description:** Initializes parallel port mode.
**Parameters:**  
Writes 3 bytes to device:
| Byte | Value | Description        |
|------|-------|--------------------|
| 0    | 0xB1  | Command code       |
| 1    | 0x02  | Mode configuration:<br>- 0x02 EPP mode |
| 2    | 0x00  | Reserved           |

---

#### CH341_CMD_GET_STATUS (0xA0)
**USB Endpoint:** BULK_OUT (1 byte) → BULK_IN (3 bytes)  
**Description:** Reads GPIO states and status flags  
**Response Format:** 3 bytes  
| Byte | Bit Range | Field Name      | Description                      |
|------|-----------|-----------------|----------------------------------|
| 0    | 7-0       | D7-D0           | GPIO data lines 0-7             |
| 1    | 8         | ERR#            | Error status pin                 |
|      | 9         | PEMP            | Parallel port empty flag         |
|      | 10        | INT#            | Interrupt status pin             |
|      | 11        | SLCT            | Chip select status               |
| 2    | 13        | BUSY/WAIT#      | Busy/Wait status pin             |
|      | 14        | AUTOFD#/DATAS#  | Auto-feed/Data strobe status     |
|      | 15        | SLCTIN#/ADDRS#  | Select-in/Address strobe status  |
|      | 23        | SDA             | I²C SDA line status (if connected)|

---

#### CH341_CMD_SET_OUTPUT (0xA1)
**USB Endpoint:** BULK_OUT (11 bytes)  
**Description:** Configures GPIO direction and output states.  
**Parameters:**  
Requires 11-byte payload:
| Byte | Value       | Description                        |
|------|-------------|------------------------------------|
| 0    | `0xA1`      | Command code                       |
| 1    | `0x6A`      | Fixed configuration value          |
| 2    | `0x1F`      | Output enable mask (bits 16-19)    |
| 3    | Data Byte 1 | GPIO data bits 15-8 (LSB first)   |
| 4    | Dir Byte 1  | GPIO direction 15-8 + `0x10` mask  |
| 5    | Data Byte 2 | GPIO data bits 7-0                |
| 6    | Dir Byte 2  | GPIO direction 7-0 + `0x10` mask   |
| 7    | Data Byte 3 | GPIO data bits 23-16              |
| 8-10 | `0x00`      | Padding (unused)                   |

---

#### CH341_CMD_SPI_STREAM (0xA8)

**USB Endpoint:** BULK_OUT (command+data) → BULK_IN (response)  
**Description:** Performs SPI transfer. First byte is command (0xA8), followed by data payload (up to 32 bytes). Response length matches TX length.  
**Parameters:**  
- Byte 0: `0xA8` (Command code)  
- Byte 1-N: SPI data payload (up to 32 bytes, LSB-first per byte)  

---

#### CH341_CMD_UIO_STREAM (0xAB)
**USB Endpoint:** BULK_OUT  
**Description:** Handles GPIO streams. Command (0xAB) followed by subcommands:

##### UIO_STM_IN (0x00)
**Description:** Reads GPIO 0-7  
**Parameters:**  
Send 3 bytes:
| Byte | Value | Description      |
|------|-------|------------------|
| 0    | 0xAB  | Stream command   |
| 1    | 0x00  | Read subcommand  |
| 2    | 0x??  | Mask (optional)  |

**Response:** 1 byte with GPIO 0-7 states

##### UIO_STM_OUT (0x80)
**Description:** Writes GPIO 0-5  
**Parameters:**  
Send 3 bytes:
| Byte | Value | Description      |
|------|-------|------------------|
| 0    | 0xAB  | Stream command   |
| 1    | 0x80  | Write subcommand |
| 2    | 0x??  | Data value (bits 0-5) |

##### UIO_STM_DIR (0x40)
**Description:** Sets GPIO direction.
**Parameters:**  
Send 3 bytes:
| Byte | Value | Description      |
|------|-------|------------------|
| 0    | 0xAB  | Stream command   |
| 1    | 0x40  | Direction subcommand    |
| 2    | 0x??  | Direction mask (bits 0-5: 0=input, 1=output) |

##### UIO_STM_END (0x20)
**Description:** Terminates stream. 
**Parameters:**  
Send 2 bytes:
| Byte | Value | Description      |
|------|-------|------------------|
| 0    | 0xAB  | Stream command   |
| 1    | 0x20  | End subcommand   |

---

#### UIO_STM_US (0xC0)
**USB Endpoint:** BULK_OUT    
**Description:** Adds delay in microseconds  
**Parameters:**  
Send 3 bytes:
| Byte | Value | Description         |
|------|-------|---------------------|
| 0    | 0xC0  | Delay command       |
| 1-2  |       | 16-bit delay value  |

---

### Notes
- All multi-byte values use little-endian byte order.

**Validation Notes:**
- All commands use 1-byte command codes
- GET_STATUS requires 3-byte read response
- SET_OUTPUT requires 11-byte write with fixed structure
- SPI_STREAM uses variable-length data payload
- UIO_STREAM subcommands follow strict 3-byte sequence
- Direction masks use 0x10 offset for internal configuration
