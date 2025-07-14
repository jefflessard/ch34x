### CH341 Command Documentation

#### 1. Parallel Port Initialization (0xB1)
**USB Endpoint:** BULK_OUT  
**Description:** Configures device for parallel operations  
**Parameters:**  
- Byte 0: `0xB1` (Command code)  
- Byte 1: `0x02` (Fixed value for EPP mode configuration)  
- Byte 2: `0x00` (Reserved, always zero)  

---

#### 2. Get Status (0xA0)
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

#### 3. Set Output (0xA1)
**USB Endpoint:** BULK_OUT (11 bytes)  
**Description:** Configures GPIO direction and output values  
**Parameters:**  
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

#### 4. SPI Stream (0xA8)
**USB Endpoint:** BULK_OUT/IN  
**Description:** Performs SPI data transfer  
**Parameters:**  
- Byte 0: `0xA8` (Command code)  
- Byte 1-N: SPI data payload (up to 32 bytes, LSB-first per byte)  

---

#### 5. UIO Stream Control (0xAB)
**USB Endpoint:** BULK_OUT  
**Subcommands:**  

##### UIO_STM_IN (0x00)
Reads GPIO 0-7  
Parameters:  
- Byte 0: `0xAB` (Stream header)  
- Byte 1: `0x00` (Read command)  
Response: 1 byte with GPIO 0-7 states

##### UIO_STM_OUT (0x80)
Writes GPIO 0-5  
Parameters:  
- Byte 0: `0xAB`  
- Byte 1: `0x80`  
- Byte 2: Data value (bits 0-5)

##### UIO_STM_DIR (0x40)
Sets GPIO direction  
Parameters:  
- Byte 0: `0xAB`  
- Byte 1: `0x40`  
- Byte 2: Direction mask (bits 0-5: 0=input, 1=output)

##### UIO_STM_END (0x20)
Terminates stream  
Parameters:  
- Byte 0: `0xAB`  
- Byte 1: `0x20`

---

#### 6. Microsecond Delay (0xC0)
**USB Endpoint:** BULK_OUT  
**Description:** Adds delay in microseconds  
**Parameters:**  
- Byte 0: `0xC0`  
- Byte 1-2: 16-bit little-endian delay value

**Validation Notes:**
- All commands use 1-byte command codes
- GET_STATUS requires 3-byte read response
- SET_OUTPUT requires 11-byte write with fixed structure
- SPI_STREAM uses variable-length data payload
- UIO_STREAM subcommands follow strict 3-byte sequence
- Direction masks use 0x10 offset for internal configuration
