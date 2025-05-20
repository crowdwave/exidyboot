Okay, I will assemble the software components into a "single whole" by providing the complete Python server script (which includes the BASIC program with the embedded Z80 first-stage loader) and the Z80 assembly language source code for the CP/M BIOS that you originally provided.

This system is designed to:

1.  Allow the Exidy Sorcerer to load a small BASIC program via audio.
2.  The BASIC program POKEs a tiny Z80 machine-language loader into RAM.
3.  This Z80 loader (using the Sorcerer's `PARLIN` ROM routine) downloads the full CP/M system (including your custom BIOS) from the Python server via the parallel port.
4.  CP/M then runs on the Sorcerer, using your custom BIOS to perform disk I/O via the parallel port to the Python server.

Here are the components:

-----

## **Component 1: Exidy Sorcerer CP/M BIOS (Z80 Assembly)**

This is the Z80 assembly code you provided in your first prompt. It implements the CP/M BIOS disk functions to communicate over the parallel port. Ensure this code is assembled and linked into your CP/M system image (`.dsk` file) that will be served as the `boot-image`.

```assembly
; ************************************************************
; CP/M PARALLEL PORT DISK DRIVER FOR EXIDY SORCERER
; (Provided by User)
; ************************************************************
;
; This driver replaces the standard disk routines in CP/M BIOS
; Handles all disk operations via parallel port to host computer
;
; Port addresses:
; FF - Data port (8-bit data transfer)
; FE - Status port (handshaking)
;
; Protocol commands from this BIOS to Server:
; 01 - Read sector
; 02 - Write sector
; 03 - Initialize disks
; 04 - Get disk status

; Status port bit definitions (as used by this Z80 BIOS code):
; For communication with the server, this BIOS interprets signals on Port 0FEh:
; - Bit 7 (DATA_READY from Server): Server indicates it has data for Sorcerer / Server acknowledges Sorcerer's data.
; - Bit 0 (ACK_BIT to Server): Sorcerer indicates it has taken data / Sorcerer indicates its data is ready.

; ************************************************************
; CONSTANTS
; ************************************************************

DATA_PORT       EQU     0FFh    ; Parallel port data register
STATUS_PORT     EQU     0FEh    ; Parallel port status register
DATA_READY      EQU     80h     ; Bit 7 of status port (Server sets this HIGH when its data is ready for Sorcerer, or to ACK Sorcerer's data)
ACK_BIT         EQU     01h     ; Bit 0 of status port (Sorcerer sets this HIGH to ACK server's data, or when its own data is ready)

; Command codes
CMD_READ        EQU     01h     ; Read sector
CMD_WRITE       EQU     02h     ; Write sector
CMD_INIT        EQU     03h     ; Initialize disk system
CMD_STATUS      EQU     04h     ; Get disk status (custom command)

; Standard CP/M sector size
SECTOR_SIZE     EQU     128     ; CP/M uses 128-byte sectors

; BIOS Entry points (standard CP/M jump table would point here)
; Example: Assumes this code is linked appropriately in a CP/M image.
; ORG appropriate_bios_start_address

; Cold Boot Entry (minimal, real CP/M boot is more complex)
BOOT:
        LD SP, 0F000h-100h ; Example stack in high RAM (adjust if RAM map differs)
                           ; This needs to be a valid RAM address for your system configuration.
        CALL    INIT_DISK  ; Initialize our parallel disk system
        ; ... other boot tasks like console init ...
        RET                ; Return to CCP/BDOS

; Warm Boot Entry
WBOOT:
        ; ... reload CCP etc ...
        RET

; CONOST: Check console status
CONOST:
        ; TODO: Implement console status check (e.g., from Sorcerer ROM)
        LD A,0FFh ; Example: no char ready = 00, char ready = FF
        RET

; CONIN: Read console character
CONIN:
        ; TODO: Implement console input (e.g., from Sorcerer ROM)
        CALL    0E006h ; Example: Call Sorcerer Monitor KEYIN
        RET

; CONOUT: Write console character
CONOUT:
        ; On entry: C = character to write
        LD A,C
        ; TODO: Implement console output (e.g., from Sorcerer ROM)
        CALL    0E003h ; Example: Call Sorcerer Monitor DISPLY
        RET

; LIST: Write character to list device
LIST:
        ; TODO: Implement list device output (can be same as CONOUT or NOP)
        RET

; PUNCH: Write character to punch device
PUNCH:
        ; NOP for this example
        RET

; READER: Read character from reader device
READER:
        ; NOP, return 0 or EOF
        XOR A
        RET

; HOME - Move to track 0 (called by BDOS)
HOME:
        LD      C,0             ; Set track to 0
        ; Fall through to SETTRK

; SETTRK - Set Track Number
; On entry: C = track number (BDOS passes track in C for some calls, BC for others, check CP/M docs)
; This implementation assumes C holds the track.
SETTRK:
        LD      A,C             ; Get low byte of track
        LD      (TRACK),A       ; Store it
        RET

; SETSEC - Set Sector Number
; On entry: C = sector number (BDOS passes sector in C or BC)
; This implementation assumes C holds the sector.
SETSEC:
        LD      A,C             ; Get sector number
        LD      (SECTOR),A      ; Store it
        RET

; SETDMA - Set DMA Address
; On entry: BC = DMA address
SETDMA:
        LD      (DMA_ADDR),BC   ; Store DMA address
        RET

; READ - Read a sector
; On exit: A = 0 if successful, non-zero if error
READ:
        ; Send READ command
        LD      A,CMD_READ
        CALL    SEND_COMMAND
        
        ; Send disk, track, sector
        LD      A,(DISKNO)
        CALL    SEND_BYTE
        LD      A,(TRACK)
        CALL    SEND_BYTE
        LD      A,(SECTOR)
        CALL    SEND_BYTE
        
        ; Read status byte (0 = success) from Server
        CALL    RECV_BYTE
        OR      A               ; Set flags
        JR      NZ,READ_ERROR   ; If not zero, return error code from server
        
        ; Read 128 bytes of sector data
        LD      HL,(DMA_ADDR)   ; Load destination address
        LD      B,SECTOR_SIZE   ; 128 bytes per sector
READ_LOOP:
        CALL    RECV_BYTE       ; Get a byte from server
        LD      (HL),A          ; Store it
        INC     HL              ; Next position
        DJNZ    READ_LOOP       ; Repeat for all bytes
        
        XOR     A               ; Return success (A=0)
        RET
        
READ_ERROR:
        ; A already contains error code from server
        RET

; WRITE - Write a sector
; On entry from BDOS: C = write type (0=normal, 1=directory, etc. - this BIOS doesn't use it beyond passing)
; On exit: A = 0 if successful, non-zero if error
WRITE:
        ; Send WRITE command
        LD      A,CMD_WRITE
        CALL    SEND_COMMAND
        
        ; Send disk, track, sector, write-type
        LD      A,(DISKNO)
        CALL    SEND_BYTE
        LD      A,(TRACK)
        CALL    SEND_BYTE
        LD      A,(SECTOR)
        CALL    SEND_BYTE
        LD      A,C             ; Write type (from original BDOS call's C reg)
        CALL    SEND_BYTE       ; Send write_type to server
        
        ; Send 128 bytes of sector data
        LD      HL,(DMA_ADDR)   ; Load source address
        LD      B,SECTOR_SIZE   ; 128 bytes per sector
WRITE_LOOP:
        LD      A,(HL)          ; Get a byte
        CALL    SEND_BYTE       ; Send it to server
        INC     HL              ; Next position
        DJNZ    WRITE_LOOP      ; Repeat for all bytes
        
        ; Read status byte (0 = success) from Server
        CALL    RECV_BYTE
        RET                     ; Return status in A (0 for success, else error)

; SELDSK - Select Disk Drive
; On entry: C = disk number (0=A, 1=B, etc)
; On exit:  HL = address of DPH, or 0000h if error
SELDSK:
        LD      A,C
        CP      2               ; Support only drives A: (0) and B: (1)
        JR      NC,SELDSK_ERR   ; If C >= 2, error
        
        LD      (DISKNO),A      ; Save selected disk
        
        ; Point HL to the correct Disk Parameter Header (DPH)
        LD      HL,DPH0         ; Default to DPH for drive A
        OR      A               ; Is it drive 0? (Test A for zero)
        RET     Z               ; Yes, return with HL pointing to DPH0
        
        LD      HL,DPH1         ; Otherwise, it's drive 1 (B:), point to DPH for drive B
        RET
        
SELDSK_ERR:
        LD      HL,0            ; Return HL=0000h for error (disk not available)
        RET

; SECTRN - Sector translation (translate logical to physical sector)
; On entry: BC = logical sector number, DE = address of translation table (XLT)
; On exit: HL = physical sector number
; This BIOS uses no translation (1:1 mapping) as it's talking to a server.
SECTRN:
        LD      H,B             ; HL = BC (no translation)
        LD      L,C
        RET

; ************************************************************
; COMMUNICATION SUBROUTINES (as per original user-provided Z80 BIOS)
; These define how this BIOS talks to the server.
; The Python server's command handling phase must match this handshake.
; ************************************************************

; SEND_COMMAND - Send a command byte to server
; On entry: A = command code
SEND_COMMAND:
        ; This routine assumes server is master for DATA_READY in this phase
        PUSH    AF              ; Save command
SEND_CMD_WAIT:                  ; Wait for server to be ready (DATA_READY from server is LOW)
        IN      A,(STATUS_PORT)
        AND     DATA_READY      ; Check Server's DATA_READY signal
        JR      NZ,SEND_CMD_WAIT ; Loop if server's DATA_READY is HIGH (server not ready for our command)
        
        POP     AF              ; Restore command
        OUT     (DATA_PORT),A   ; Send command byte to server
        
        LD      A,ACK_BIT       ; Sorcerer's ACK_BIT = 01h
        OUT     (STATUS_PORT),A ; Assert our ACK_BIT (signal to server: "my command/data is on the port")
        
WAIT_CMD_ACK:                   ; Wait for server to acknowledge by setting DATA_READY HIGH
        IN      A,(STATUS_PORT)
        AND     DATA_READY
        JR      Z,WAIT_CMD_ACK  ; Loop if server's DATA_READY is LOW
        
        XOR     A               ; A = 0
        OUT     (STATUS_PORT),A ; Clear our ACK_BIT
        RET

; SEND_BYTE - Send a data byte to server (used after SEND_COMMAND)
; On entry: A = byte to send
SEND_BYTE:
        PUSH    AF
SEND_BYTE_WAIT:                 ; Wait for server's DATA_READY to be LOW
        IN      A,(STATUS_PORT)
        AND     DATA_READY
        JR      NZ,SEND_BYTE_WAIT
        
        POP     AF
        OUT     (DATA_PORT),A   ; Send data byte
        
        LD      A,ACK_BIT
        OUT     (STATUS_PORT),A ; Assert our ACK_BIT
        
WAIT_BYTE_ACK:                  ; Wait for server's DATA_READY to be HIGH
        IN      A,(STATUS_PORT)
        AND     DATA_READY
        JR      Z,WAIT_BYTE_ACK
        
        XOR     A
        OUT     (STATUS_PORT),A ; Clear our ACK_BIT
        RET

; RECV_BYTE - Receive a byte from server
; On exit: A = received byte
; This matches Sorcerer PARLIN protocol
RECV_BYTE:
RECV_WAIT:                      ; Wait for server to assert DATA_READY HIGH
        IN      A,(STATUS_PORT)
        AND     DATA_READY
        JR      Z,RECV_WAIT
        
        IN      A,(DATA_PORT)   ; Read the data byte from server
        PUSH    AF              ; Save it
        
        LD      A,ACK_BIT
        OUT     (STATUS_PORT),A ; Assert our ACK_BIT (acknowledge to server)
        
RECV_ACK_WAIT:                  ; Wait for server to de-assert DATA_READY LOW
        IN      A,(STATUS_PORT)
        AND     DATA_READY
        JR      NZ,RECV_ACK_WAIT ; Loop if server's DATA_READY is still HIGH
        
        XOR     A
        OUT     (STATUS_PORT),A ; Clear our ACK_BIT
        
        POP     AF              ; Restore received byte
        RET

; ************************************************************
; DISK INITIALIZATION (Called from BOOT)
; ************************************************************
INIT_DISK:
        LD      A,CMD_INIT
        CALL    SEND_COMMAND    ; Send INIT command to server
        
        CALL    RECV_BYTE       ; Get status from server
        ; A should be 0 if successful from server
        RET                     ; Return status in A

; ************************************************************
; DISK PARAMETER BLOCKS AND HEADERS (DPH, DPB)
; ************************************************************

; Disk Parameter Header for drive A (first drive)
DPH0:
        DW      0               ; Translation vector (XLT) - 0 for no translation
        DW      0,0,0           ; BDOS scratch area (16-bits each, 3 words)
        DW      DIRBUF          ; Directory buffer pointer
        DW      DPB0            ; Disk parameter block pointer for drive A
        DW      CHK0            ; Checksum vector pointer (for checking directory)
        DW      ALL0            ; Allocation vector pointer (for disk space)

; Disk Parameter Header for drive B (second drive)
DPH1:
        DW      0               ; XLT
        DW      0,0,0           ; Scratch
        DW      DIRBUF          ; Shared Directory buffer
        DW      DPB0            ; Shared Disk parameter block (can be different if drives differ)
        DW      CHK1            ; Checksum vector for drive B
        DW      ALL1            ; Allocation vector for drive B

; Disk Parameter Block (DPB) for Standard 8" SSSD CP/M like disk
; This example DPB is for drive A. Drive B could have DPB1 if different.
DPB0:
        DW      26              ; SPT - Sectors per track
        DB      3               ; BSH - Block shift factor (2K blocks: 2^(3+7) = 2^10 = 1024 bytes/block)
        DB      7               ; BLM - Block mask (2K blocks: 2^3-1 = 7)
        DB      0               ; EXM - Extent mask (standard 8" has 0)
        DW      242             ; DSM - Disk size - 1 (Total 243 blocks on disk)
                                ; (243 blocks * 1024 bytes/block = 248832 bytes ~ 243KB)
        DW      63              ; DRM - Directory entries - 1 (64 directory entries)
        DB      192             ; AL0 - Directory allocation bitmap byte 0 (11000000b for 64 entries in 2K blocks)
        DB      0               ; AL1 - Directory allocation bitmap byte 1
        DW      16              ; CKS - Checksum size (for directory, 16 entries for 64/4)
                                ; (Number of directory entries that are checksummed, if 0, all are)
                                ; Or, size of directory check area in records. If 0, all dir entries have checksums.
                                ; Often set to (DRM+1)/4 for CP/M 2.2
        DW      2               ; OFF - Track offset (number of reserved tracks)

; ************************************************************
; BIOS DATA STORAGE AREA
; ************************************************************
DISKNO:         DB      0       ; Currently selected disk (0=A, 1=B)
TRACK:          DB      0       ; Current track
SECTOR:         DB      0       ; Current sector
DMA_ADDR:       DW      0100h   ; Default DMA address (CP/M TPA start)

; Buffers and vectors required by CP/M (sizes depend on DPB values)
DIRBUF:         DS      SECTOR_SIZE ; Directory buffer (128 bytes)

; Checksum vectors (size CKS from DPB, number of directory entries to buffer for checksumming)
; CKS=16 means 16 directory entries, each 32 bytes = 512 bytes. This seems too large.
; CKS is often (MaxDirEntries/4) or 0. If 0, all dir entries have software checksums.
; If CKS is "size of directory check area in records (128 bytes)", so 16*128 bytes.
; Let's assume CKS = (DRM+1)/NumRecInBlock = 64 / (1024/128) = 64/8 = 8. So 8 * 128 = 1K.
; Or, if CKS is # of directory entries that have checksums stored with them.
; For simplicity, let's use a small fixed size for these example buffers if they are scratch.
; The DPH points to these; CP/M uses them. The sizes must be correct.
; DRM = 63 (64 entries). (64 entries * 32 bytes/entry) / 128 bytes/sector = 16 sectors for directory.
; CHK0/CHK1 are usually scratch space for BDOS related to checksumming. Size related to CKS in DPB.
; DPB's CKS is "size of checksum vector in records if non-zero, or number of dir entries to check if zero"
; A common size for CHK is 16 bytes if CKS in DPB is 0, or if CKS refers to a directory check area size in 128-byte records.
; Let's assume small scratch areas.
CHK0:           DS      16      ; Checksum scratch for drive A
CHK1:           DS      16      ; Checksum scratch for drive B

; Allocation vectors (size related to DSM from DPB, (DSM+1)/8 bytes)
; DSM = 242 (243 blocks). (243+1)/8 = 244/8 = 30.5 -> 31 bytes.
ALL0:           DS      31      ; Allocation vector for drive A (for 243 blocks)
ALL1:           DS      31      ; Allocation vector for drive B

        END
```

-----

## **Component 2: Python Server (`cpm_server.py`)**

This is the complete Python script. It includes the `BASIC_BOOTLOADER` string with the corrected 30-byte Z80 first-stage loader (using `PARLIN`, `START=$0D00`, `SP=$0CFF`).

```python
#!/usr/bin/env python3
"""
CP/M Disk Server with Continuous BASIC Bootloader Audio for Exidy Sorcerer
This server:
1. Continuously transmits a BASIC bootloader (which uses Sorcerer ROM PARLIN) via audio.
2. Waits for the Sorcerer (running the first-stage loader) to request the CP/M system.
3. Sends the CP/M system image over the parallel port.
4. After CP/M boots on the Sorcerer, serves CP/M disk I/O requests via parallel port,
   matching the handshake of the user-provided Z80 BIOS.

Exidy Sorcerer Parallel Port (as per user-provided technical info):
- Data Port: 0xFF
- Status Port: 0xFE
  - Bit 7 (Input to Sorcerer): "Data Available" from Server/Peripheral.
  - Bit 0 (Output from Sorcerer): "Data Acknowledge" by Sorcerer.
  - ROM Input Routine: PARLIN ($E01E) uses this handshake for Sorcerer receiving.

User's Z80 CP/M BIOS Handshake (Sorcerer Sending to Server via its SEND_BYTE/SEND_COMMAND):
1. Sorc: Waits for Server's "Data Available" (Sorc Port FE bit 7) to be LOW.
2. Sorc: OUT (0xFF), A (sends byte).
3. Sorc: LD A, 0x01; OUT (0xFE), A (sets Sorc Port FE bit 0 HIGH - its "data is out/ACK").
4. Sorc: Waits for Server's "Data Available" (Sorc Port FE bit 7) to be HIGH (Server "ACKs receipt of data").
5. Sorc: XOR A; OUT (0xFE), A (sets Sorc Port FE bit 0 LOW - clears its "data is out/ACK").
(The Python server's `_server_receives_byte_from_sorcerer_z80bios` matches this)

User's Z80 CP/M BIOS Handshake (Sorcerer Receiving from Server via its RECV_BYTE):
 This RECV_BYTE routine in the Z80 BIOS itself directly matches the PARLIN handshake.
 Python server's `_server_sends_byte_to_sorcerer` handles this.
"""

import time
import sys
import argparse
import os
import struct
import logging
from threading import Thread, Lock
import wave # Ensure wave is imported
import numpy as np
import pyaudio # Ensure pyaudio is imported

try:
    import parallel
except ImportError:
    print("Error: pyparallel library not found. Install with 'pip install pyparallel'")
    print("Note: On modern systems, you may need additional drivers/permissions for parallel port access.")
    sys.exit(1)

# --- BASIC Bootloader Definition (using PARLIN $E01E, START at $0D00, SP at $0CFF) ---
# Z80 Machine Code (30 bytes): F3 31 FF 0C CD 1E E0 4F CD 1E E0 47 21 00 01 78 B1 28 08 CD 1E E0 77 23 0B 18 F4 C3 00 01
BASIC_LOADER_DEC_BYTES_RAM = [
    243, 49, 255, 12,      # F3 31 FF 0C (DI; LD SP,0CFFh)
    205, 30, 224,          # CD 1E E0 (CALL E01Eh PARLIN)
    79,                    # 4F       (LD C,A)
    205, 30, 224,          # CD 1E E0 (CALL E01Eh PARLIN)
    71,                    # 47       (LD B,A)
    33, 0, 1,              # 21 00 01 (LD HL,0100h)
    120,                   # 78       (LD A,B) ; LOAD_LOOP_HEAD
    177,                   # B1       (OR C)
    40, 8,                 # 28 08    (JR Z to JP 0100h)
    205, 30, 224,          # CD 1E E0 (CALL E01Eh PARLIN)
    119,                   # 77       (LD (HL),A)
    35,                    # 23       (INC HL)
    11,                    # 0B       (DEC BC)
    24, 244,               # 18 F4    (JR to LOAD_LOOP_HEAD)
    195, 0, 1              # C3 00 01 (JP 0100h)
] # Total 30 bytes

basic_data_lines_ram = []
current_line_num_ram = 40
bytes_per_line_ram = 8 
for i in range(0, len(BASIC_LOADER_DEC_BYTES_RAM), bytes_per_line_ram):
    chunk = BASIC_LOADER_DEC_BYTES_RAM[i:i+bytes_per_line_ram]
    basic_data_lines_ram.append(f"{current_line_num_ram} DATA {','.join(map(str, chunk))}")
    current_line_num_ram += 10
num_loader_bytes_ram = len(BASIC_LOADER_DEC_BYTES_RAM)

LOADER_START_ADDRESS_DEC = 3328 # $0D00 Hex

BASIC_BOOTLOADER_PROGRAM_LINES_RAM = [
    "10 REM PARALLEL BOOTLOADER (V3-PARLIN, RAM) FOR EXIDY SORCERER",
    f"20 START = {LOADER_START_ADDRESS_DEC} : REM M/L START ADDR (HEX {LOADER_START_ADDRESS_DEC:04X})",
    f"30 LOADER_LEN = {num_loader_bytes_ram} : REM M/L LENGTH ({num_loader_bytes_ram} BYTES)",
] + basic_data_lines_ram + [
    f"{current_line_num_ram} REM POKE BOOTLOADER INTO RAM AT 'START'",
    f"{current_line_num_ram + 10} PRINT \"POKING \"; LOADER_LEN; \" BYTES TO \"; START",
    f"{current_line_num_ram + 20} AD = START",
    f"{current_line_num_ram + 30} FOR I = 1 TO LOADER_LEN",
    f"{current_line_num_ram + 40} READ B : POKE AD, B : AD = AD + 1", # Combined for brevity
    f"{current_line_num_ram + 50} NEXT I",
    f"{current_line_num_ram + 60} PRINT \"MACHINE CODE LOADED.\"",
    f"{current_line_num_ram + 70} PRINT \"NOW LOADING CP/M VIA PARALLEL PORT...\"",
    f"{current_line_num_ram + 80} X=USR(START)", 
    f"{current_line_num_ram + 90} PRINT \"USR CALL RETURNED. CP/M SHOULD BE RUNNING.\"",
    f"{current_line_num_ram + 100} PRINT \"TYPE SYSTEM OR CP/M COMMANDS.\"",
    f"{current_line_num_ram + 110} END"
]
BASIC_BOOTLOADER = "\n".join(BASIC_BOOTLOADER_PROGRAM_LINES_RAM)

# --- Logging Configuration ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s',
                    handlers=[logging.FileHandler("cpm_server.log", mode='w'), logging.StreamHandler()])
logger = logging.getLogger(__name__)

# --- Constants ---
CPM_SECTOR_SIZE = 128
CPM_SECTORS_PER_TRACK = 26 
CPM_TRACKS_PER_DISK = 77   
SYSTEM_SIZE_TO_SEND = 16 * 1024 

CMD_READ = 0x01
CMD_WRITE = 0x02
CMD_INIT = 0x03
CMD_STATUS = 0x04

SAMPLE_RATE = 44100
BAUD_RATE = 1200

# --- Parallel Port Handshake Pin/Bit Mappings (CRITICAL USER CONFIGURATION) ---
# **USER MUST VERIFY THESE LAMBDAS AGAINST THEIR PC LPT PORT WIRING TO SORCERER DB25 PINS.**
# Sorcerer Port 0xFE, Bit 7 (Input): "Data Available" (from Server). High means data available from Server.
SERVER_SETS_DATA_AVAILABLE_ASSERT   = lambda p: p.setAutofeed(1) # PC Control Port ~AUTOFEED (Pin 14) assumed for this
SERVER_SETS_DATA_AVAILABLE_DEASSERT = lambda p: p.setAutofeed(0)

# Sorcerer Port 0xFE, Bit 0 (Output): "Data Acknowledge" (from Sorcerer). High means Sorcerer Acknowledged.
SORCERER_ACK_BIT_ON_PC_STATUS_MASK    = 0x01 # Assumes Sorcerer's ACK -> PC Status Port Bit 0 (non-standard for PC)
SERVER_READS_SORCERER_ACK_ASSERTED    = lambda p: (p.getStatus() & SORCERER_ACK_BIT_ON_PC_STATUS_MASK) != 0
SERVER_READS_SORCERER_ACK_DEASSERTED  = lambda p: (p.getStatus() & SORCERER_ACK_BIT_ON_PC_STATUS_MASK) == 0
# --- End Critical User Configuration for Handshake Pins ---

class AudioBootloader(Thread):
    def __init__(self):
        Thread.__init__(self, name="AudioBootloaderThread", daemon=True)
        self.running = True; self.audio_interface = None; self.audio_stream = None; self.audio_bytes = b''
        self._prepare_audio()

    def _prepare_audio(self):
        logger.info("Preparing BASIC bootloader audio...")
        self.audio_interface = pyaudio.PyAudio()
        leader_samples = self._generate_tone(1200, 3.0)
        basic_program_bytes = self._convert_basic_text_to_tape_format_bytes(BASIC_BOOTLOADER)
        kcs_payload_samples = self._encode_data_to_kcs_samples(basic_program_bytes)
        trailer_samples = self._generate_tone(1200, 1.0)
        if kcs_payload_samples.size == 0: logger.warning("KCS payload is empty.")
        self.audio_data_np = np.concatenate([leader_samples, kcs_payload_samples, trailer_samples])
        self.audio_bytes = (self.audio_data_np * 32767).astype(np.int16).tobytes()
        logger.info(f"BASIC bootloader audio prepared ({len(self.audio_data_np)/SAMPLE_RATE:.2f}s).")

    def _convert_basic_text_to_tape_format_bytes(self, basic_text):
        # WARNING: Simplified format. Real Sorcerer tape files are more complex.
        filename = b'BOOTMLV3'; file_type = bytes([0x01])
        program_bytes = (basic_text.strip() + '\r').encode('ascii')
        return filename + file_type + struct.pack("<H", len(program_bytes)) + program_bytes

    def _generate_tone(self, freq, dur):
        t = np.linspace(0, dur, int(SAMPLE_RATE * dur), endpoint=False)
        return np.sin(freq * t * 2 * np.pi).astype(np.float32)

    def _encode_data_to_kcs_samples(self, data_bytes):
        bd = 1.0 / BAUD_RATE; samples = []
        for byte_val in data_bytes:
            samples.append(self._generate_tone(1200, bd)) # Start bit
            for i in range(8): samples.append(self._generate_tone(2400 if (byte_val>>i)&1 else 1200, bd))
            samples.append(self._generate_tone(2400, bd)); samples.append(self._generate_tone(2400, bd)) # Stop bits
        return np.concatenate(samples) if samples else np.array([], dtype=np.float32)

    def run(self):
        logger.info("Audio bootloader thread started.")
        try:
            self.audio_stream = self.audio_interface.open(format=pyaudio.paInt16,channels=1,rate=SAMPLE_RATE,output=True)
            while self.running:
                if self.audio_bytes: self.audio_stream.write(self.audio_bytes)
                time.sleep(0.5)
        except Exception as e: logger.error(f"AudioBootloader error: {e}",exc_info=True)
        finally:
            if self.audio_stream: self.audio_stream.stop_stream();self.audio_stream.close()
            if self.audio_interface: self.audio_interface.terminate()
            logger.info("Audio bootloader thread stopped.")
    def stop(self): self.running = False; logger.debug("Audio stop request.")

class CPMDiskServer:
    def __init__(self, port_spec, disk_paths=None, boot_img_path=None):
        try:
            self.port = parallel.Parallel(port_spec)
            logger.info(f"Parallel port '{port_spec}' initialized.")
        except Exception as e: logger.critical(f"Init port '{port_spec}': {e}",exc_info=True); sys.exit(1)
        self.disk_files = []
        if disk_paths:
            for i,p in enumerate(disk_paths[:2]):
                try:
                    if not os.path.exists(p): logger.warning(f"Disk {chr(65+i)} img missing: {p}")
                    self.disk_files.append(open(p, "r+b"))
                    logger.info(f"Opened disk {chr(65+i)}: {p}")
                except Exception as e: logger.error(f"Opening disk {p}: {e}")
        self.boot_image_path = boot_img_path
        self.comm_lock = Lock(); self.audio_thread = AudioBootloader(); self.timeout = 15.0

    def _calc_offset(self, trk, sec): # Track 0-based, Sector 1-based
        if not (0<=trk<CPM_TRACKS_PER_DISK and 1<=sec<=CPM_SECTORS_PER_TRACK): raise ValueError(f"Bad T{trk}/S{sec}")
        return (trk * CPM_SECTORS_PER_TRACK + (sec - 1)) * CPM_SECTOR_SIZE

    # --- Server SENDING data to Sorcerer (Sorcerer uses PARLIN or its own RECV_BYTE) ---
    def _server_sends_byte_to_sorcerer(self, byte_val):
        with self.comm_lock:
            # logger.debug(f"S->S: Send 0x{byte_val:02X}. Wait Sorc ACK LOW.")
            self._wait_sorc_ack_is_deasserted() # PARLIN clears ACK before reading next
            self.port.setData(byte_val)
            SERVER_SETS_DATA_AVAILABLE_ASSERT(self.port) # Server: "Data is ready"
            # logger.debug(f"S->S: Data 0x{byte_val:02X} on port, DataAvail HIGH. Wait Sorc ACK HIGH.")
            self._wait_sorc_ack_is_asserted()    # Sorcerer: "Got it" (sets its ACK)
            SERVER_SETS_DATA_AVAILABLE_DEASSERT(self.port) # Server: "OK, Data no longer guaranteed"
            # logger.debug(f"S->S: DataAvail LOW. Wait Sorc ACK LOW (cycle end).")
            self._wait_sorc_ack_is_deasserted()  # Sorcerer: "Ready for next" (clears its ACK)
            # logger.debug(f"S->S: Sent 0x{byte_val:02X} OK.")

    # --- Server RECEIVING data from Sorcerer (Sorcerer Z80 BIOS uses SEND_BYTE/SEND_COMMAND) ---
    def _server_receives_byte_from_sorcerer_z80bios(self):
        with self.comm_lock:
            # logger.debug("S<-S: Prep recv. Server DataAvail LOW.")
            SERVER_SETS_DATA_AVAILABLE_DEASSERT(self.port) # 1. Server ensures its DataAvail is LOW.
            
            # logger.debug("S<-S: Wait Sorc ACK HIGH (Sorc sent byte + its ACK).")
            self._wait_sorc_ack_is_asserted()      # 2. Wait for Sorc's ACK to go HIGH.
            
            byte_val = self.port.getData()         # 3. Read data.
            # logger.debug(f"S<-S: Got 0x{byte_val:02X}. Server DataAvail HIGH (my ACK).")

            SERVER_SETS_DATA_AVAILABLE_ASSERT(self.port) # 4. Server sets its DataAvail HIGH (as its ACK).
            
            # logger.debug("S<-S: Wait Sorc ACK LOW (Sorc saw my ACK, clears its own).")
            self._wait_sorc_ack_is_deasserted()  # 5. Wait for Sorc's ACK to go LOW.
            
            SERVER_SETS_DATA_AVAILABLE_DEASSERT(self.port)# 6. Server clears its DataAvail.
            # logger.debug(f"S<-S: Recvd 0x{byte_val:02X} OK from Z80BIOS.")
            return byte_val

    # Handshake Helpers (using current SERVER_PIN/SORCERER_ACK defs)
    def _wait_sorc_ack_is_asserted(self):
        start_t = time.time()
        while not SERVER_READS_SORCERER_ACK_ASSERTED(self.port):
            if time.time() - start_t > self.timeout: raise TimeoutError("Wait Sorc ACK Asserted")
            time.sleep(0.00001)
    def _wait_sorc_ack_is_deasserted(self):
        start_t = time.time()
        while not SERVER_READS_SORCERER_ACK_DEASSERTED(self.port):
            if time.time() - start_t > self.timeout: raise TimeoutError("Wait Sorc ACK Deasserted")
            time.sleep(0.00001)

    def perform_initial_boot_transfer(self):
        if not self.boot_image_path or not os.path.exists(self.boot_image_path):
            logger.error(f"Boot image '{self.boot_image_path}' missing. Cannot boot."); return False
        logger.info(f"Starting CP/M boot transfer from '{self.boot_image_path}', {SYSTEM_SIZE_TO_SEND} bytes.")
        time.sleep(0.5) 
        try:
            len_msb, len_lsb = (SYSTEM_SIZE_TO_SEND >> 8) & 0xFF, SYSTEM_SIZE_TO_SEND & 0xFF
            logger.info(f"Sending length: MSB=0x{len_msb:02X}, LSB=0x{len_lsb:02X}");
            self._server_sends_byte_to_sorcerer(len_msb)
            self._server_sends_byte_to_sorcerer(len_lsb)
            logger.info("Sending CP/M system data...")
            sent_count = 0
            with open(self.boot_image_path, "rb") as f:
                while sent_count < SYSTEM_SIZE_TO_SEND:
                    byte = f.read(1)
                    val = byte[0] if byte else 0xE5 # Pad with 0xE5 (CP/M empty) if image short
                    if not byte and sent_count == os.path.getsize(self.boot_image_path) : # Log padding once
                        logger.warning(f"Boot image ended at {sent_count} bytes. Padding with 0xE5.")
                    self._server_sends_byte_to_sorcerer(val)
                    sent_count += 1
                    if sent_count % 2048 == 0: logger.info(f"Sent {sent_count}/{SYSTEM_SIZE_TO_SEND} boot bytes...")
            logger.info(f"CP/M boot transfer of {sent_count} bytes complete.")
            return True
        except TimeoutError: logger.error("Timeout during boot transfer."); return False
        except Exception as e: logger.error(f"Boot transfer error: {e}",exc_info=True); return False

    def _bios_cmd_read(self):
        d,t,s = (self._server_receives_byte_from_sorcerer_z80bios() for _ in range(3))
        logger.info(f"CMD: READ D={d}, T={t}, S={s}")
        stat, data = 1, [0xE5]*CPM_SECTOR_SIZE
        if d < len(self.disk_files) and self.disk_files[d]:
            try:
                self.disk_files[d].seek(self._calc_offset(t,s))
                r_data = self.disk_files[d].read(CPM_SECTOR_SIZE)
                data = list(r_data) + [0xE5]*(CPM_SECTOR_SIZE - len(r_data)); stat=0
            except Exception as e: logger.error(f"READ DSK ERR: {e}",exc_info=True); stat=2
        else: logger.error(f"READ Invalid Disk {d}"); stat=1
        self._server_sends_byte_to_sorcerer(stat)
        if stat==0: [self._server_sends_byte_to_sorcerer(b) for b in data]
        logger.debug(f"READ done. Status:{stat}")

    def _bios_cmd_write(self):
        d,t,s,wt = (self._server_receives_byte_from_sorcerer_z80bios() for _ in range(4))
        logger.info(f"CMD: WRITE D={d}, T={t}, S={s}, Type={wt}")
        stat=1; data=[self._server_receives_byte_from_sorcerer_z80bios() for _ in range(CPM_SECTOR_SIZE)]
        if d < len(self.disk_files) and self.disk_files[d]:
            try:
                self.disk_files[d].seek(self._calc_offset(t,s))
                self.disk_files[d].write(bytes(data)); self.disk_files[d].flush(); stat=0
            except Exception as e: logger.error(f"WRITE DSK ERR: {e}",exc_info=True); stat=2
        else: logger.error(f"WRITE Invalid Disk {d}"); stat=1
        self._server_sends_byte_to_sorcerer(stat)
        logger.debug(f"WRITE done. Status:{stat}")

    def run_bios_command_server(self):
        logger.info("CP/M BIOS Command Server started...")
        try:
            while True:
                try:
                    cmd = self._server_receives_byte_from_sorcerer_z80bios()
                    if cmd == CMD_READ: self._bios_cmd_read()
                    elif cmd == CMD_WRITE: self._bios_cmd_write()
                    elif cmd == CMD_INIT: logger.info("CMD: INIT"); self._server_sends_byte_to_sorcerer(0)
                    elif cmd == CMD_STATUS: logger.info("CMD: STATUS"); self._server_sends_byte_to_sorcerer(0)
                    else: logger.warning(f"UNKNOWN BIOS cmd: 0x{cmd:02X}")
                except TimeoutError: logger.warning("Timeout in BIOS cmd loop. Awaiting next.")
                except Exception as e: logger.error(f"BIOS cmd loop error: {e}",exc_info=True); time.sleep(0.1)
        except KeyboardInterrupt: logger.info("BIOS Command Server stopped by user.")
        finally: self.shutdown()

    def start_services(self):
        self.audio_thread.start()
        if not self.perform_initial_boot_transfer():
            logger.critical("Initial boot transfer FAILED. Server shutting down."); self.shutdown(); sys.exit(1)
        self.run_bios_command_server()

    def shutdown(self):
        logger.info("Shutting down CPMDiskServer...");
        if hasattr(self,'audio_thread') and self.audio_thread.is_alive(): self.audio_thread.stop(); self.audio_thread.join(1)
        for f in self.disk_files:
            try: f.close()
            except: pass
        logger.info("Server shutdown complete.")

def main():
    parser = argparse.ArgumentParser(
        description='CP/M Disk Server for Exidy Sorcerer.',
        formatter_class=argparse.RawTextHelpFormatter,
        epilog=f"""
CRITICAL WIRING & PYPARALLEL ASSUMPTIONS:
The Python script uses specific `pyparallel` calls to control/read PC LPT port lines.
These MUST be correctly wired to the Exidy Sorcerer's DB25 parallel port pins.
Default assumptions in this script (these may need to be changed in the Python code):
1. Server controls "Data Available" to Sorcerer (Sorcerer Port 0xFE bit 7 / Sorcerer Pin 9 "Input data available"):
   - Asserted HIGH by: `SERVER_SETS_DATA_AVAILABLE_ASSERT(port)` -> `port.setAutofeed(1)`
   - De-asserted LOW by: `SERVER_SETS_DATA_AVAILABLE_DEASSERT(port)` -> `port.setAutofeed(0)`
   (This implies PC LPT Control Pin 14 [~AUTOFEED, active LOW output] is wired to Sorcerer Pin 9,
    and an inverter is used OR `setAutofeed(1)` makes PC Pin 14 HIGH directly if pyparallel/OS allows).

2. Server reads Sorcerer's "Data Acknowledge" (Sorcerer Port 0xFE bit 0 / Sorcerer Pin 2 "Output data accepted"):
   - Asserted HIGH by Sorcerer: `SERVER_READS_SORCERER_ACK_ASSERTED(port)` -> `(port.getStatus() & {SORCERER_ACK_BIT_ON_PC_STATUS_MASK}) != 0`
   - De-asserted LOW by Sorcerer: `SERVER_READS_SORCERER_ACK_DEASSERTED(port)` -> `(port.getStatus() & {SORCERER_ACK_BIT_ON_PC_STATUS_MASK}) == 0`
   (This assumes Sorcerer Pin 2 is wired to a PC LPT Status Port line that appears as bit {SORCERER_ACK_BIT_ON_PC_STATUS_MASK}
    [default 0x01, i.e., bit 0] in the byte read by `port.getStatus()`. PC LPT Status Port bit 0 is non-standard.)

**VERIFY YOUR WIRING and adjust the `SERVER_SETS_*` and `SERVER_READS_*` lambda functions at the top of the script if needed.**
The Z80 M/L loader is POKEd by BASIC into RAM at $0D00 ({LOADER_START_ADDRESS_DEC} decimal).
The Z80 M/L loader uses Sorcerer Monitor ROM routine PARLIN ($E01E) for byte input.
""")
    parser.add_argument('--port', type=str, default='0x378', help='Parallel port (0x378, /dev/parport0). Default: 0x378')
    parser.add_argument('--boot-image', type=str, required=True, help='Path to CP/M boot disk image.')
    parser.add_argument('--disks', type=str, nargs='*', help='(Optional) Paths to disk images for A:, B:.')
    parser.add_argument('--save-audio', type=str, metavar='FILE.wav', help='Save BASIC loader audio to WAV and exit.')
    parser.add_argument('--verbose', '-v', action='store_true', help='Enable DEBUG logging.')
    
    args = parser.parse_args()
    if args.verbose: logger.setLevel(logging.DEBUG); [h.setLevel(logging.DEBUG) for h in logger.handlers]
    
    if args.save_audio:
        try:
            audio_s = AudioBootloader(); wave.open(args.save_audio,'w').setparams((1,2,SAMPLE_RATE,0,'NONE','not_compressed')).writeframes(audio_s.audio_bytes); audio_s.stop(); audio_s.join(1)
            logger.info(f"Bootloader audio saved to {args.save_audio}")
        except Exception as e: logger.error(f"Audio save error: {e}",exc_info=True)
        return

    port_spec = args.port if args.port.startswith('/dev/') else int(args.port,0)
    server = None
    try:
        server = CPMDiskServer(port_spec, args.disks, args.boot_image)
        server.start_services()
    except Exception: logger.critical("Unhandled exception at main level.", exc_info=True)
    finally:
        if server: server.shutdown()
        logger.info("Main: Server terminated.")

if __name__ == "__main__":
    main()
```

-----

## **Component 3: Integration and Usage Instructions**

1.  **Prepare CP/M Boot Image:**

      * Take your Exidy Sorcerer CP/M disk image (e.g., `sorcerer_boot.dsk`).
      * Assemble the Z80 BIOS code (Component 1 above) into machine code (`.hex` or `.bin`).
      * Use CP/M tools (like `SYSGEN` or a disk image editor) to replace the existing BIOS in your `sorcerer_boot.dsk` with your newly assembled parallel port BIOS. Ensure it's placed at the correct memory location where CP/M expects to find the BIOS. The first 16KB of this modified `sorcerer_boot.dsk` will be sent by the Python server.

2.  **Hardware Connection (CRITICAL):**

      * Connect your PC's parallel port (LPT) to the Exidy Sorcerer's DB25 parallel interface.
      * **Verify Wiring:** The Python script makes assumptions about which PC LPT pins (controlled/read by `pyparallel` functions like `setAutofeed`, `getStatus`) are connected to the Sorcerer's specific handshake pins (Pin 9 "Input data available", Pin 2 "Output data accepted", etc.).
          * **You MUST check your cable and potentially adjust the `SERVER_SETS_DATA_AVAILABLE_ASSERT/DEASSERT` and `SERVER_READS_SORCERER_ACK_ASSERTED/DEASSERTED` lambda functions at the top of the Python script.** The `epilog` in the `argparse` help text provides the current assumptions. Incorrect wiring or `pyparallel` mapping for these handshake lines is the most likely cause of communication failure.
      * Connect the PC's audio output to the Sorcerer's tape input jack.

3.  **Run the Python Server:**

      * Save the Python code above as `cpm_server.py`.
      * Install necessary Python libraries: `pip install pyparallel numpy pyaudio`.
      * Run from your terminal:
        ```bash
        python cpm_server.py --boot-image path/to/your/sorcerer_boot.dsk --disks path/to/your/sorcerer_boot.dsk path/to/other_disk.dsk --port YOUR_LPT_PORT --verbose
        ```
        Replace `path/to/...` with your actual file paths.
        Replace `YOUR_LPT_PORT` with your PC's parallel port (e.g., `0x378` for LPT1 on Windows/DOS, or `/dev/parport0` on Linux).
        The `--verbose` flag is highly recommended for debugging.

4.  **Load and Run on Exidy Sorcerer:**

      * Turn on the Sorcerer.
      * Once the Python server is running and the audio is playing (you should see logs or hear it if your speakers are on), type `LOAD` (or the equivalent tape load command for your Sorcerer Monitor ROM) on the Sorcerer.
      * The `BASIC_BOOTLOADER` program should load from the audio. The Sorcerer will likely display "FOUND BOOTMLV3" or similar.
      * After it loads, type `RUN` on the Sorcerer.
      * The BASIC program will:
          * Print "POKING 30 BYTES TO 3328".
          * POKE the 30-byte Z80 first-stage loader into RAM at address `$0D00` (3328 decimal).
          * Print "MACHINE CODE LOADED."
          * Print "NOW LOADING CP/M VIA PARALLEL PORT..."
          * Execute `X=USR(3328)`.
      * The Z80 first-stage loader will now run. It will communicate with the Python server to download the 16KB CP/M system (from your `boot-image`) into Sorcerer RAM starting at address `$0100`.
      * If all handshaking and data transfer are successful, the Z80 loader will jump to `$0100`, starting CP/M.
      * You should then see the CP/M prompt (e.g., `A>`) on the Sorcerer. Subsequent disk operations will use your custom Z80 BIOS, communicating with the Python server.

This provides all the software components and a roadmap. The parallel port wiring and its correct reflection in the `pyparallel` calls within the Python script are the most delicate parts requiring your verification.
