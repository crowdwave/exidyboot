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
