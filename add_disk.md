# Built-in CP/M Disk Manager for Exidy Sorcerer

That's a fantastic suggestion! We can integrate disk management directly into CP/M without requiring a separate program. Here's how to implement this approach:

## 1. CP/M BIOS Extension

Instead of creating a standalone program, we'll extend the CP/M BIOS to include disk management capabilities. This makes the functionality always available without loading anything from disk.

```assembly
; ************************************************************
; CP/M BIOS Extension for Disk Image Management
; (Add to the existing BIOS code)
; ************************************************************

; Additional command codes for server communication
CMD_DISK_CATALOG EQU     09h     ; Get disk image catalog
CMD_DISK_EJECT   EQU     0Bh     ; Eject current disk
CMD_DISK_INSERT  EQU     0Ch     ; Insert new disk

; ************************************************************
; SELDSK - Modified to handle special disk management commands
; ************************************************************
SELDSK:
        LD      A,C             ; Get drive number
        CP      4               ; Is it drive E: or higher?
        JR      C,SELDSK_NORMAL ; If A-D, handle normally
        
        ; Special drive handling
        CP      12              ; Is it drive M:?
        JP      Z,DISK_MANAGER  ; If M:, run disk manager
        
        ; Invalid drive
        LD      HL,0            ; Return 0 for error
        RET

SELDSK_NORMAL:
        ; Original SELDSK code for drives A-D
        ; [existing code remains unchanged]

; ************************************************************
; DISK_MANAGER - Built-in disk management functions
; ************************************************************
DISK_MANAGER:
        ; Save registers
        PUSH    BC
        PUSH    DE
        
        ; Clear screen
        CALL    CLS
        
        ; Display banner
        LD      DE,DM_BANNER
        CALL    PRTSTR
        
DISK_MENU:
        ; Display menu
        LD      DE,DM_MENU
        CALL    PRTSTR
        
        ; Get choice
        CALL    CONIN
        
        ; Process choice
        CP      'C'
        JP      Z,DM_CATALOG
        CP      'c'
        JP      Z,DM_CATALOG
        
        CP      'E'
        JP      Z,DM_EJECT
        CP      'e'
        JP      Z,DM_EJECT
        
        CP      'I'
        JP      Z,DM_INSERT
        CP      'i'
        JP      Z,DM_INSERT
        
        CP      'Q'
        JP      Z,DM_EXIT
        CP      'q'
        JP      Z,DM_EXIT
        
        ; Invalid choice
        LD      DE,DM_INVALID
        CALL    PRTSTR
        JP      DISK_MENU

; Display disk catalog from server
DM_CATALOG:
        ; Call server to get catalog
        LD      A,CMD_DISK_CATALOG
        CALL    SEND_CMD
        
        ; Check for error
        OR      A
        JP      NZ,DM_COMM_ERROR
        
        ; Get status
        CALL    RECV_BYTE
        OR      A
        JP      NZ,DM_DISK_ERROR
        
        ; Get count of images (16-bit)
        CALL    RECV_BYTE       ; Low byte
        LD      C,A
        CALL    RECV_BYTE       ; High byte
        LD      B,A
        
        ; Store count
        LD      (DM_IMG_COUNT),BC
        
        ; Display catalog header
        LD      DE,DM_CAT_HEADER
        CALL    PRTSTR
        
        ; If no images, show message
        LD      A,B
        OR      C
        JP      Z,DM_NO_IMAGES
        
        ; Display image count
        LD      A,'['
        CALL    CONOUT
        CALL    PRINT_NUMBER
        LD      A,']'
        CALL    CONOUT
        
        ; Display images
        LD      B,0             ; Counter
        
DM_CAT_LOOP:
        ; Show item number
        LD      A,B
        ADD     A,'1'
        CALL    CONOUT
        LD      A,'.'
        CALL    CONOUT
        LD      A,' '
        CALL    CONOUT
        
        ; Get filename (null-terminated)
        LD      HL,DM_FILENAME
        CALL    RECV_STRING
        
        ; Display filename
        LD      DE,DM_FILENAME
        CALL    PRTSTR
        
        LD      A,' '
        CALL    CONOUT
        LD      A,'('
        CALL    CONOUT
        
        ; Get format (null-terminated)
        LD      HL,DM_FORMAT
        CALL    RECV_STRING
        
        ; Display format
        LD      DE,DM_FORMAT
        CALL    PRTSTR
        
        LD      A,')'
        CALL    CONOUT
        CALL    CRLF
        
        ; Store filename in catalog 
        LD      HL,DM_CATALOG
        CALL    ADD_TO_CATALOG
        
        ; Increment counter
        INC     B
        
        ; Check if we've displayed all images
        LD      A,(DM_IMG_COUNT)
        CP      B
        JP      NZ,DM_CAT_LOOP
        
        ; Done
        JP      DISK_MENU

; Eject a disk
DM_EJECT:
        ; Prompt for drive
        LD      DE,DM_DRIVE_PROMPT
        CALL    PRTSTR
        
        ; Get drive letter
        CALL    CONIN
        
        ; Convert to uppercase
        CALL    TO_UPPER
        
        ; Check valid range (A-D)
        CP      'A'
        JP      C,DM_INV_DRIVE
        CP      'E'
        JP      NC,DM_INV_DRIVE
        
        ; Convert to drive number
        SUB     'A'
        LD      (DM_SEL_DRIVE),A
        
        ; Confirm
        LD      DE,DM_CONFIRM
        CALL    PRTSTR
        
        CALL    CONIN
        CALL    TO_UPPER
        
        CP      'Y'
        JP      NZ,DISK_MENU    ; User canceled
        
        ; Send eject command
        LD      A,CMD_DISK_EJECT
        CALL    SEND_CMD
        
        ; Check for error
        OR      A
        JP      NZ,DM_COMM_ERROR
        
        ; Send drive number
        LD      A,(DM_SEL_DRIVE)
        CALL    SEND_BYTE
        
        ; Get status
        CALL    RECV_BYTE
        OR      A
        JP      NZ,DM_DISK_ERROR
        
        ; Success
        LD      DE,DM_EJECT_OK
        CALL    PRTSTR
        
        ; Wait for keypress
        CALL    CONIN
        JP      DISK_MENU

; Insert a disk
DM_INSERT:
        ; First show catalog
        CALL    DM_CATALOG
        
        ; Prompt for drive
        LD      DE,DM_DRIVE_PROMPT
        CALL    PRTSTR
        
        ; Get drive letter
        CALL    CONIN
        
        ; Convert to uppercase
        CALL    TO_UPPER
        
        ; Check valid range (A-D)
        CP      'A'
        JP      C,DM_INV_DRIVE
        CP      'E'
        JP      NC,DM_INV_DRIVE
        
        ; Convert to drive number
        SUB     'A'
        LD      (DM_SEL_DRIVE),A
        
        ; Prompt for image number
        LD      DE,DM_IMG_PROMPT
        CALL    PRTSTR
        
        ; Get image number
        CALL    CONIN
        
        ; Convert to 0-based index
        SUB     '1'
        JP      C,DM_INV_IMAGE
        
        ; Check range
        LD      HL,(DM_IMG_COUNT)
        CP      L
        JP      NC,DM_INV_IMAGE
        
        ; Save selected image
        LD      (DM_SEL_IMAGE),A
        
        ; Look up filename
        CALL    GET_FILENAME
        
        ; Show confirmation
        LD      DE,DM_INS_CONF1
        CALL    PRTSTR
        
        LD      DE,DM_FILENAME
        CALL    PRTSTR
        
        LD      DE,DM_INS_CONF2
        CALL    PRTSTR
        
        ; Show drive letter
        LD      A,(DM_SEL_DRIVE)
        ADD     A,'A'
        CALL    CONOUT
        
        LD      DE,DM_INS_CONF3
        CALL    PRTSTR
        
        ; Get confirmation
        CALL    CONIN
        CALL    TO_UPPER
        
        CP      'Y'
        JP      NZ,DISK_MENU    ; User canceled
        
        ; Send insert command
        LD      A,CMD_DISK_INSERT
        CALL    SEND_CMD
        
        ; Check for error
        OR      A
        JP      NZ,DM_COMM_ERROR
        
        ; Send drive number
        LD      A,(DM_SEL_DRIVE)
        CALL    SEND_BYTE
        
        ; Get filename length
        LD      HL,DM_FILENAME
        LD      B,0
DM_GET_LEN:
        LD      A,(HL)
        OR      A
        JP      Z,DM_GOT_LEN
        INC     HL
        INC     B
        JP      DM_GET_LEN
DM_GOT_LEN:
        
        ; Send length
        LD      A,B
        CALL    SEND_BYTE
        
        ; Send filename
        LD      HL,DM_FILENAME
        LD      B,0
DM_SEND_NAME:
        LD      A,(HL)
        OR      A
        JP      Z,DM_SENT_NAME
        CALL    SEND_BYTE
        INC     HL
        JP      DM_SEND_NAME
DM_SENT_NAME:
        
        ; Get status
        CALL    RECV_BYTE
        OR      A
        JP      NZ,DM_DISK_ERROR
        
        ; Success
        LD      DE,DM_INSERT_OK
        CALL    PRTSTR
        
        ; Wait for keypress
        CALL    CONIN
        JP      DISK_MENU

; Error handlers
DM_INV_DRIVE:
        LD      DE,DM_INV_DRIVE_MSG
        CALL    PRTSTR
        CALL    CONIN
        JP      DISK_MENU

DM_INV_IMAGE:
        LD      DE,DM_INV_IMG_MSG
        CALL    PRTSTR
        CALL    CONIN
        JP      DISK_MENU

DM_NO_IMAGES:
        LD      DE,DM_NO_IMG_MSG
        CALL    PRTSTR
        CALL    CONIN
        JP      DISK_MENU

DM_COMM_ERROR:
        LD      DE,DM_COMM_ERR_MSG
        CALL    PRTSTR
        CALL    CONIN
        JP      DISK_MENU

DM_DISK_ERROR:
        LD      B,A             ; Save error code
        LD      DE,DM_DISK_ERR_MSG
        CALL    PRTSTR
        
        ; Show error code
        LD      A,B
        CALL    PRTHEX
        
        CALL    CONIN
        JP      DISK_MENU

; Exit disk manager
DM_EXIT:
        LD      DE,DM_EXIT_MSG
        CALL    PRTSTR
        
        ; Restore registers
        POP     DE
        POP     BC
        
        ; Return with error (no disk selected)
        LD      HL,0
        RET

; Utility routines (Add these to your BIOS)
; --------------------------------------------

; Receive a null-terminated string into HL
RECV_STRING:
        PUSH    BC
        LD      B,64            ; Maximum length
RECV_STR_LOOP:
        CALL    RECV_BYTE
        LD      (HL),A
        INC     HL
        OR      A
        JP      Z,RECV_STR_DONE ; Null terminator found
        DJNZ    RECV_STR_LOOP
        
        ; Force termination if too long
        DEC     HL
        LD      (HL),0
RECV_STR_DONE:
        POP     BC
        RET

; Store filename in catalog
ADD_TO_CATALOG:
        PUSH    BC
        PUSH    DE
        PUSH    HL
        
        ; Calculate position in catalog
        LD      A,B             ; Current index
        LD      HL,DM_CATALOG
        
        ; Multiply index by 33 (32 chars + null)
        LD      C,33
        LD      E,0
        LD      D,0
MULT_LOOP:
        OR      A
        JP      Z,MULT_DONE
        ADD     HL,DE
        DEC     A
        JP      MULT_LOOP
MULT_DONE:
        
        ; Copy filename to catalog
        LD      DE,DM_FILENAME
        LD      BC,33
        LDIR
        
        POP     HL
        POP     DE
        POP     BC
        RET

; Get filename from catalog
GET_FILENAME:
        PUSH    BC
        PUSH    DE
        
        ; Calculate position in catalog
        LD      A,(DM_SEL_IMAGE)
        LD      HL,DM_CATALOG
        
        ; Multiply index by 33 (32 chars + null)
        LD      C,33
        LD      E,0
        LD      D,0
GET_MULT_LOOP:
        OR      A
        JP      Z,GET_MULT_DONE
        ADD     HL,DE
        DEC     A
        JP      GET_MULT_LOOP
GET_MULT_DONE:
        
        ; Copy to filename buffer
        LD      DE,DM_FILENAME
        LD      BC,33
        LDIR
        
        POP     DE
        POP     BC
        RET

; Convert A to uppercase
TO_UPPER:
        CP      'a'
        RET     C
        CP      'z'+1
        RET     NC
        SUB     32
        RET

; Print 16-bit number in BC
PRINT_NUMBER:
        ; [Add code to print decimal number]
        RET

; Print hex value of A
PRTHEX:
        PUSH    AF
        PUSH    BC
        
        ; High nibble
        LD      C,A
        RRA
        RRA
        RRA
        RRA
        AND     0FH
        ADD     A,'0'
        CP      '9'+1
        JP      C,PRTHX1
        ADD     A,7
PRTHX1:
        CALL    CONOUT
        
        ; Low nibble
        LD      A,C
        AND     0FH
        ADD     A,'0'
        CP      '9'+1
        JP      C,PRTHX2
        ADD     A,7
PRTHX2:
        CALL    CONOUT
        
        POP     BC
        POP     AF
        RET

; Clear screen
CLS:
        LD      A,1Ah           ; ASCII Form Feed
        JP      CONOUT

; Print CR+LF
CRLF:
        LD      A,0Dh
        CALL    CONOUT
        LD      A,0Ah
        JP      CONOUT

; Print string at DE
PRTSTR:
        PUSH    BC
        PUSH    DE
        PUSH    HL
        
        LD      C,9             ; BDOS print string function
        CALL    5               ; Call BDOS
        
        POP     HL
        POP     DE
        POP     BC
        RET

; Data section
DM_BANNER:      DB  'CP/M Disk Image Manager', 13, 10, '$'
DM_MENU:        DB  13, 10, 'C - Catalog', 13, 10
                DB  'E - Eject disk', 13, 10
                DB  'I - Insert disk', 13, 10
                DB  'Q - Quit', 13, 10
                DB  13, 10, 'Enter choice: $'
DM_INVALID:     DB  13, 10, 'Invalid choice!$'
DM_CAT_HEADER:  DB  13, 10, 'Available disk images: $'
DM_DRIVE_PROMPT: DB 13, 10, 'Enter drive (A-D): $'
DM_IMG_PROMPT:  DB  13, 10, 'Enter image number: $'
DM_CONFIRM:     DB  13, 10, 'Are you sure? (Y/N) $'
DM_EJECT_OK:    DB  13, 10, 'Disk ejected successfully. Press any key.$'
DM_INSERT_OK:   DB  13, 10, 'Disk inserted successfully. Press any key.$'
DM_INS_CONF1:   DB  13, 10, 'Insert disk image "$'
DM_INS_CONF2:   DB  '" into drive $'
DM_INS_CONF3:   DB  '? (Y/N) $'
DM_INV_DRIVE_MSG: DB 13, 10, 'Invalid drive! Press any key.$'
DM_INV_IMG_MSG: DB  13, 10, 'Invalid image number! Press any key.$'
DM_NO_IMG_MSG:  DB  13, 10, 'No disk images available! Press any key.$'
DM_COMM_ERR_MSG: DB 13, 10, 'Communication error! Press any key.$'
DM_DISK_ERR_MSG: DB 13, 10, 'Disk operation error! Code: $'
DM_EXIT_MSG:    DB  13, 10, 'Exiting disk manager.$'

; Variables
DM_IMG_COUNT:   DW  0           ; Number of images
DM_SEL_DRIVE:   DB  0           ; Selected drive
DM_SEL_IMAGE:   DB  0           ; Selected image

; Buffers
DM_FILENAME:    DS  33          ; Buffer for current filename (32 chars + null)
DM_FORMAT:      DS  17          ; Buffer for format name (16 chars + null)
DM_CATALOG:     DS  8*33        ; Catalog buffer (up to 8 entries)
```

## 2. Accessing the Disk Manager

There are three elegant ways to access this built-in disk manager:

### Option 1: Special Drive Letter

Modify the BIOS to interpret a specific drive letter as the disk manager command:

```
M:
```

When the user types this at the CP/M prompt, it tries to select drive M:, but our modified SELDSK routine intercepts this and launches the disk manager instead.

### Option 2: Special BDOS Function

Extend the BDOS with a custom function for disk management:

```
DISKMGR
```

Create a tiny COM file (DISKMGR.COM) that just calls our custom BDOS function:

```assembly
ORG 100H
MVI C,50      ; Custom BDOS function
CALL 5        ; Call BDOS
RET           ; Return to CP/M
```

### Option 3: Reserved Filename

Modify the CCP (Console Command Processor) to intercept a special command:

```
DISKMAN
```

When the user types this command, the CCP recognizes it as special and calls our disk manager code directly.

## 3. Server-Side Implementation

The server side remains similar to the previous implementation but receives commands directly from the BIOS:

```python
def handle_disk_catalog(self):
    """Handle request for disk image catalog."""
    logger.info("Sending disk image catalog")
    
    # Scan disk images directory
    disk_dir = self.images_directory
    available_images = []
    
    for filename in os.listdir(disk_dir):
        if filename.endswith(('.img', '.dsk', '.cpm')):
            image_path = os.path.join(disk_dir, filename)
            size = os.path.getsize(image_path)
            format_name = "Unknown"
            
            # Try to detect format
            with open(image_path, 'rb') as f:
                detected_format = self._detect_disk_format(f)
                if detected_format:
                    format_name = detected_format.name
            
            available_images.append({
                'name': filename,
                'path': image_path,
                'size': size,
                'format': format_name
            })
    
    # Send success status
    self.send_byte(ERROR_NONE)
    
    # Send count of images (limited to 255 for simplicity)
    count = min(len(available_images), 255)
    self.send_byte(count & 0xFF)
    self.send_byte((count >> 8) & 0xFF)
    
    # Send image entries
    for img in available_images[:count]:
        # Send filename (null-terminated)
        for c in img['name'][:32]:
            self.send_byte(ord(c))
        self.send_byte(0)  # Null terminator
        
        # Send format name (null-terminated)
        for c in img['format'][:16]:
            self.send_byte(ord(c))
        self.send_byte(0)  # Null terminator
    
    logger.info(f"Sent catalog with {count} entries")
```

## 4. Advantages of the Built-in Approach

1. **Always Available**: No need to load a program from disk
2. **Memory Efficient**: Uses code already in the BIOS
3. **Seamless Integration**: Feels like a natural part of CP/M
4. **Universal Access**: Works from any drive, even if A: has been ejected
5. **Robust**: Works even when disks are corrupted or missing

## 5. User Experience

From the user's perspective, this is seamless:

1. User types `M:` at the CP/M prompt
2. Disk manager menu appears
3. User can browse disk catalog, eject and insert disks
4. After quitting, user is back at the CP/M prompt
5. All operations are handled on the server, keeping the Sorcerer side lightweight

This implementation gives you the best of both worlds - the convenience of built-in disk management with most of the processing handled by the server.
