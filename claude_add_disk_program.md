# Implementing Disk Image Catalog and Swapping for the Exidy Sorcerer

That's an excellent enhancement idea! I can implement a disk image catalog and hot-swapping system that would allow the Sorcerer to browse and select from available disk images on the server. Here's how we can make this work:

## 1. Protocol Extensions

First, we'll add new commands to our communication protocol:

```python
# Disk management commands
CMD_DISK_CATALOG = 0x09       # Get disk image catalog
CMD_DISK_INFO = 0x0A          # Get detailed disk image info
CMD_DISK_EJECT = 0x0B         # Eject current disk
CMD_DISK_INSERT = 0x0C        # Insert new disk
```

## 2. Server-Side Implementation

Let's enhance the server to:

1. **Scan a disk images directory** for available images
2. **Provide a catalog** when requested
3. **Track mounted disks** and allow hot-swapping

Here's the code for the server-side implementation:

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
            # Try to detect format
            format_name = "Unknown"
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
    
    # Send count of images
    count = len(available_images)
    self.send_byte(count & 0xFF)
    self.send_byte((count >> 8) & 0xFF)
    
    # Send image entries
    for img in available_images:
        # Send filename (null-terminated)
        for c in img['name'][:32]:  # Limit to 32 chars
            self.send_byte(ord(c))
        self.send_byte(0)  # Null terminator
        
        # Send format name (null-terminated)
        for c in img['format'][:16]:  # Limit to 16 chars
            self.send_byte(ord(c))
        self.send_byte(0)  # Null terminator
    
    logger.info(f"Sent catalog with {count} entries")

def handle_disk_eject(self):
    """Handle disk eject command."""
    # Get drive number
    drive = self.receive_byte()
    if drive is None or drive >= len(self.disk_files):
        self.send_byte(ERROR_INVALID_DISK)
        return
    
    logger.info(f"Ejecting disk from drive {chr(65+drive)}:")
    
    # Close file
    try:
        self.disk_files[drive].close()
        self.disk_files[drive] = None
        self.disk_images[drive] = None
        
        # Send success
        self.send_byte(ERROR_NONE)
        logger.info(f"Disk ejected from drive {chr(65+drive)}:")
    except Exception as e:
        logger.error(f"Error ejecting disk: {e}")
        self.send_byte(ERROR_UNKNOWN)

def handle_disk_insert(self):
    """Handle disk insert command."""
    # Get drive number
    drive = self.receive_byte()
    if drive is None or drive >= len(self.disk_files):
        self.send_byte(ERROR_INVALID_DISK)
        return
    
    # Receive filename length
    name_len = self.receive_byte()
    if name_len is None or name_len > 64:
        self.send_byte(ERROR_UNKNOWN)
        return
    
    # Receive filename
    filename = ""
    for _ in range(name_len):
        c = self.receive_byte()
        if c is None:
            self.send_byte(ERROR_TIMEOUT)
            return
        filename += chr(c)
    
    image_path = os.path.join(self.images_directory, filename)
    logger.info(f"Inserting disk {filename} into drive {chr(65+drive)}:")
    
    # Check if file exists
    if not os.path.exists(image_path):
        logger.error(f"Disk image not found: {image_path}")
        self.send_byte(ERROR_FILE_NOT_FOUND)
        return
    
    # Close current disk file if open
    if self.disk_files[drive] is not None:
        self.disk_files[drive].close()
    
    # Open new disk file
    try:
        disk_file = open(image_path, "r+b")
        self.disk_files[drive] = disk_file
        self.disk_images[drive] = image_path
        
        # Detect format
        detected_format = self._detect_disk_format(disk_file)
        if detected_format:
            self.disk_formats[drive] = detected_format
            logger.info(f"Detected format: {detected_format.name}")
        
        # Send success
        self.send_byte(ERROR_NONE)
        logger.info(f"Disk inserted into drive {chr(65+drive)}:")
    except Exception as e:
        logger.error(f"Error inserting disk: {e}")
        self.send_byte(ERROR_UNKNOWN)
```

## 3. CP/M Program for the Sorcerer

Now, we'll create a CP/M program in Z80 assembly that provides a user interface for browsing and selecting disk images:

```assembly
; ************************************************************
; DISKMGR - Disk Image Management Program for Exidy CP/M
; ************************************************************

        org     0100h           ; Standard CP/M program origin

; CP/M BDOS entry point
BDOS    equ     0005h
; BDOS functions
CONOUT  equ     2               ; Console output
CONIN   equ     1               ; Console input
PRINT   equ     9               ; Print string

; Our custom commands
CMD_DISK_CATALOG equ    09h     ; Get disk image catalog
CMD_DISK_EJECT   equ    0Bh     ; Eject current disk
CMD_DISK_INSERT  equ    0Ch     ; Insert new disk

; Parallel port constants (same as our bootstrap loader)
DATA_PORT       equ     0FFh    ; Data port
STATUS_PORT     equ     0FEh    ; Status port
DATA_READY      equ     80h     ; Bit 7: Data Ready
BUSY_BIT        equ     40h     ; Bit 6: Busy
ACK_BIT         equ     01h     ; Bit 0: Acknowledge

; Program start
start:
        ; Display welcome message
        ld      de,welcome_msg
        call    print_string

main_menu:
        ; Display menu
        ld      de,menu_msg
        call    print_string
        
        ; Get user choice
        call    get_key
        
        ; Check choices
        cp      'C'
        jp      z,catalog_command
        cp      'c'
        jp      z,catalog_command
        
        cp      'E'
        jp      z,eject_command
        cp      'e'
        jp      z,eject_command
        
        cp      'I'
        jp      z,insert_command
        cp      'i'
        jp      z,insert_command
        
        cp      'Q'
        jp      z,exit_program
        cp      'q'
        jp      z,exit_program
        
        ; Invalid choice, show menu again
        ld      de,invalid_msg
        call    print_string
        jp      main_menu

; Get disk catalog from server and display it
catalog_command:
        ; Clear the screen
        ld      de,cls_str
        call    print_string
        
        ; Display header
        ld      de,catalog_header
        call    print_string
        
        ; Send catalog command
        ld      a,CMD_DISK_CATALOG
        call    send_command
        
        ; Check for error
        or      a
        jp      nz,comm_error
        
        ; Get status
        call    recv_byte
        or      a
        jp      nz,disk_error
        
        ; Get count of images (16-bit)
        call    recv_byte       ; Low byte
        ld      c,a
        call    recv_byte       ; High byte
        ld      b,a
        
        ; BC now has count of images
        ld      a,b
        or      c
        jp      z,no_images
        
        ; Store count for later
        ld      (image_count),bc
        
        ; Print count
        ld      a,'['
        call    print_char
        ld      hl,number_buffer
        call    word_to_dec
        ld      de,number_buffer
        call    print_string
        ld      a,']'
        call    print_char
        call    print_newline
        
        ; Read and display image entries
        ld      b,0             ; Image index counter
        
read_next_image:
        ; Display index
        ld      a,b
        add     a,'1'           ; Convert to 1-based number
        call    print_char
        ld      a,'.'
        call    print_char
        ld      a,' '
        call    print_char
        
        ; Read filename (null-terminated)
        ld      hl,filename_buffer
        ld      c,32            ; Max length
read_filename_loop:
        call    recv_byte
        ld      (hl),a
        inc     hl
        or      a               ; Check for null terminator
        jp      z,filename_done
        dec     c
        jp      nz,read_filename_loop
        ; Force termination if too long
        ld      (hl),0
filename_done:
        ; Print filename
        ld      de,filename_buffer
        call    print_string
        
        ; Print separator
        ld      a,' '
        call    print_char
        ld      a,'('
        call    print_char
        
        ; Read format (null-terminated)
        ld      hl,format_buffer
        ld      c,16            ; Max length
read_format_loop:
        call    recv_byte
        ld      (hl),a
        inc     hl
        or      a               ; Check for null terminator
        jp      z,format_done
        dec     c
        jp      nz,read_format_loop
        ; Force termination if too long
        ld      (hl),0
format_done:
        ; Print format
        ld      de,format_buffer
        call    print_string
        
        ld      a,')'
        call    print_char
        call    print_newline
        
        ; Store filename in our catalog (for later selection)
        ld      hl,catalog_buffer
        ld      a,b             ; Current index
        ld      c,33            ; 33 bytes per entry (32 + null)
        call    multiply_a_c    ; HL = A * C
        add     hl,de           ; HL now points to entry in catalog
        
        ; Copy filename to catalog
        ld      de,filename_buffer
        ld      bc,33
        ldir
        
        ; Increment image counter
        inc     b
        
        ; Check if we've read all images
        ld      hl,(image_count)
        ld      a,l
        cp      b
        jp      nz,read_next_image
        
        ; Display prompt
        call    print_newline
        ld      de,catalog_prompt
        call    print_string
        call    get_key
        
        ; Return to main menu
        jp      main_menu

; Ask for drive and eject disk
eject_command:
        ; Ask for drive
        ld      de,drive_prompt
        call    print_string
        
        ; Get drive letter (A-D)
        call    get_key
        
        ; Convert to uppercase
        cp      'a'
        jp      c,check_drive
        cp      'z'+1
        jp      nc,check_drive
        sub     32              ; Convert to uppercase
        
check_drive:
        ; Check if valid drive (A-D)
        cp      'A'
        jp      c,invalid_drive
        cp      'E'             ; Above D
        jp      nc,invalid_drive
        
        ; Convert to drive number (0-3)
        sub     'A'
        ld      b,a             ; Save drive number
        
        ; Confirm
        push    bc
        ld      de,confirm_msg
        call    print_string
        call    get_key
        pop     bc
        
        cp      'Y'
        jp      z,do_eject
        cp      'y'
        jp      z,do_eject
        
        ; Canceled
        ld      de,canceled_msg
        call    print_string
        jp      main_menu
        
do_eject:
        ; Send eject command
        ld      a,CMD_DISK_EJECT
        call    send_command
        
        ; Check for error
        or      a
        jp      nz,comm_error
        
        ; Send drive number
        ld      a,b
        call    send_byte
        
        ; Get status
        call    recv_byte
        or      a
        jp      nz,disk_error
        
        ; Success
        ld      de,eject_success
        call    print_string
        
        call    wait_key        ; Wait for a key press
        jp      main_menu

; Ask for drive and image to insert
insert_command:
        ; First get catalog (we need this for selection)
        call    catalog_command
        
        ; Ask for drive
        ld      de,drive_prompt
        call    print_string
        
        ; Get drive letter (A-D)
        call    get_key
        
        ; Convert to uppercase
        cp      'a'
        jp      c,ins_check_drive
        cp      'z'+1
        jp      nc,ins_check_drive
        sub     32              ; Convert to uppercase
        
ins_check_drive:
        ; Check if valid drive (A-D)
        cp      'A'
        jp      c,invalid_drive
        cp      'E'             ; Above D
        jp      nc,invalid_drive
        
        ; Convert to drive number (0-3)
        sub     'A'
        ld      (selected_drive),a
        
        ; Ask for image number
        ld      de,image_prompt
        call    print_string
        
        ; Get image number (1-based)
        call    get_key
        
        ; Convert to number
        sub     '1'             ; Convert to 0-based
        jp      c,invalid_image
        
        ; Check if in range
        ld      hl,(image_count)
        ld      h,0             ; We only support up to 255 images
        cp      l
        jp      nc,invalid_image
        
        ; Save selected image
        ld      (selected_image),a
        
        ; Get filename from catalog
        ld      c,33            ; 33 bytes per entry
        call    multiply_a_c    ; HL = A * C
        ld      de,catalog_buffer
        add     hl,de           ; HL now points to selected filename
        
        ; Display confirmation
        ld      de,insert_confirm_1
        call    print_string
        ld      de,hl           ; DE = filename
        call    print_string
        ld      de,insert_confirm_2
        call    print_string
        
        ; Display drive letter
        ld      a,(selected_drive)
        add     a,'A'
        call    print_char
        
        ld      de,insert_confirm_3
        call    print_string
        
        ; Get confirmation
        call    get_key
        
        cp      'Y'
        jp      z,do_insert
        cp      'y'
        jp      z,do_insert
        
        ; Canceled
        ld      de,canceled_msg
        call    print_string
        jp      main_menu
        
do_insert:
        ; Send insert command
        ld      a,CMD_DISK_INSERT
        call    send_command
        
        ; Check for error
        or      a
        jp      nz,comm_error
        
        ; Send drive number
        ld      a,(selected_drive)
        call    send_byte
        
        ; Get filename
        ld      a,(selected_image)
        ld      c,33
        call    multiply_a_c
        ld      de,catalog_buffer
        add     hl,de
        
        ; Get filename length
        ld      b,0
        push    hl
get_filename_len:
        ld      a,(hl)
        or      a
        jp      z,got_filename_len
        inc     hl
        inc     b
        jp      get_filename_len
got_filename_len:
        pop     hl              ; Restore filename pointer
        
        ; Send filename length
        ld      a,b
        call    send_byte
        
        ; Send filename
        ld      b,a             ; B = length
send_filename_loop:
        ld      a,(hl)
        call    send_byte
        inc     hl
        djnz    send_filename_loop
        
        ; Get status
        call    recv_byte
        or      a
        jp      nz,disk_error
        
        ; Success
        ld      de,insert_success
        call    print_string
        
        call    wait_key        ; Wait for a key press
        jp      main_menu

; Error handlers
invalid_drive:
        ld      de,invalid_drive_msg
        call    print_string
        call    wait_key
        jp      main_menu
        
invalid_image:
        ld      de,invalid_image_msg
        call    print_string
        call    wait_key
        jp      main_menu
        
no_images:
        ld      de,no_images_msg
        call    print_string
        call    wait_key
        jp      main_menu
        
comm_error:
        ld      de,comm_error_msg
        call    print_string
        call    wait_key
        jp      main_menu
        
disk_error:
        ; A contains error code
        ld      b,a             ; Save error code
        
        ld      de,disk_error_msg
        call    print_string
        
        ; Display error code
        ld      a,b
        call    print_hex
        
        call    wait_key
        jp      main_menu

; Exit program
exit_program:
        ; Display exit message
        ld      de,exit_msg
        call    print_string
        
        ; Return to CP/M
        ret

; Utility routines

; Print a string using CP/M BDOS
print_string:
        ld      c,PRINT
        call    BDOS
        ret

; Print a single character
print_char:
        push    de
        push    bc
        ld      e,a
        ld      c,CONOUT
        call    BDOS
        pop     bc
        pop     de
        ret

; Print a newline
print_newline:
        push    af
        ld      a,13
        call    print_char
        ld      a,10
        call    print_char
        pop     af
        ret

; Get a key from console
get_key:
        ld      c,CONIN
        call    BDOS
        ret

; Wait for a key press
wait_key:
        ld      de,press_key_msg
        call    print_string
        call    get_key
        ret

; Convert A to hex and print
print_hex:
        push    af
        push    bc
        ld      c,a
        
        ; High nibble
        rra
        rra
        rra
        rra
        call    nibble_to_hex
        call    print_char
        
        ; Low nibble
        ld      a,c
        call    nibble_to_hex
        call    print_char
        
        pop     bc
        pop     af
        ret

; Convert low nibble of A to hex character
nibble_to_hex:
        and     0Fh
        add     a,'0'
        cp      '9'+1
        jr      c,nh_done
        add     a,'A'-'9'-1
nh_done:
        ret

; Convert word in BC to decimal string at HL
word_to_dec:
        push    af
        push    bc
        push    de
        push    hl
        
        ld      de,10000
        call    div_bc_de
        add     a,'0'
        ld      (hl),a
        inc     hl
        
        ld      de,1000
        call    div_bc_de
        add     a,'0'
        ld      (hl),a
        inc     hl
        
        ld      de,100
        call    div_bc_de
        add     a,'0'
        ld      (hl),a
        inc     hl
        
        ld      de,10
        call    div_bc_de
        add     a,'0'
        ld      (hl),a
        inc     hl
        
        ld      a,c
        add     a,'0'
        ld      (hl),a
        inc     hl
        
        ld      (hl),0          ; Null terminator
        
        pop     hl
        
        ; Skip leading zeros
trim_zeros:
        ld      a,(hl)
        cp      '0'
        jr      nz,done_trim
        inc     hl
        jr      trim_zeros
        
done_trim:
        ; If we trimmed all digits, show at least a zero
        ld      a,(hl)
        or      a
        jr      nz,done_trim2
        dec     hl
        ld      (hl),'0'
        inc     hl
        ld      (hl),0
        
done_trim2:
        pop     de
        pop     bc
        pop     af
        ret

; Divide BC by DE, result in A, remainder in BC
div_bc_de:
        ld      a,0
div_loop:
        push    hl
        ld      h,b
        ld      l,c
        or      a               ; Clear carry
        sbc     hl,de
        jr      c,div_done
        ld      b,h
        ld      c,l
        inc     a
        pop     hl
        jr      div_loop
div_done:
        pop     hl
        ret

; Multiply A by C, result in HL
multiply_a_c:
        ld      hl,0
        or      a
        ret     z               ; Return 0 if A = 0
        
mult_loop:
        add     hl,bc
        dec     a
        jr      nz,mult_loop
        ret

; Send a command with error handling
send_command:
        ; Save command
        push    af
        
        ; Check if device is busy
        in      a,(STATUS_PORT)
        and     BUSY_BIT
        jr      z,send_cmd_ready
        
        ; Device is busy, error
        pop     af
        ld      a,ERROR_TIMEOUT
        ret
        
send_cmd_ready:
        ; Send command
        pop     af
        out     (DATA_PORT),a
        
        ; Set DATA_READY
        ld      a,DATA_READY
        out     (STATUS_PORT),a
        
        ; Wait for ACK
        ld      b,200           ; Timeout counter
wait_ack:
        in      a,(STATUS_PORT)
        and     ACK_BIT
        jr      nz,got_ack
        
        djnz    wait_ack
        
        ; Timeout
        xor     a
        out     (STATUS_PORT),a ; Clear DATA_READY
        ld      a,ERROR_TIMEOUT
        ret
        
got_ack:
        ; Clear DATA_READY
        xor     a
        out     (STATUS_PORT),a
        
        ; Wait for ACK to clear
        ld      b,200           ; Timeout counter
wait_ack_clear:
        in      a,(STATUS_PORT)
        and     ACK_BIT
        jr      z,ack_cleared
        
        djnz    wait_ack_clear
        
        ; Timeout
        ld      a,ERROR_TIMEOUT
        ret
        
ack_cleared:
        ; Success
        xor     a
        ret

; Send a byte with error handling
send_byte:
        push    bc
        
        ; Save byte
        ld      b,a
        
        ; Check if device is busy
        in      a,(STATUS_PORT)
        and     BUSY_BIT
        jr      z,send_byte_ready
        
        ; Device is busy, error
        pop     bc
        ld      a,ERROR_TIMEOUT
        ret
        
send_byte_ready:
        ; Send byte
        ld      a,b
        out     (DATA_PORT),a
        
        ; Set DATA_READY
        ld      a,DATA_READY
        out     (STATUS_PORT),a
        
        ; Wait for ACK
        ld      b,200           ; Timeout counter
sb_wait_ack:
        in      a,(STATUS_PORT)
        and     ACK_BIT
        jr      nz,sb_got_ack
        
        djnz    sb_wait_ack
        
        ; Timeout
        xor     a
        out     (STATUS_PORT),a ; Clear DATA_READY
        pop     bc
        ld      a,ERROR_TIMEOUT
        ret
        
sb_got_ack:
        ; Clear DATA_READY
        xor     a
        out     (STATUS_PORT),a
        
        ; Wait for ACK to clear
        ld      b,200           ; Timeout counter
sb_wait_ack_clear:
        in      a,(STATUS_PORT)
        and     ACK_BIT
        jr      z,sb_ack_cleared
        
        djnz    sb_wait_ack_clear
        
        ; Timeout
        pop     bc
        ld      a,ERROR_TIMEOUT
        ret
        
sb_ack_cleared:
        ; Success
        pop     bc
        xor     a
        ret

; Receive a byte with error handling
recv_byte:
        push    bc
        
        ; Wait for DATA_READY
        ld      b,200           ; Timeout counter
rb_wait_data:
        in      a,(STATUS_PORT)
        and     DATA_READY
        jr      nz,rb_data_ready
        
        djnz    rb_wait_data
        
        ; Timeout
        pop     bc
        ld      a,ERROR_TIMEOUT
        ret
        
rb_data_ready:
        ; Read data
        in      a,(DATA_PORT)
        push    af              ; Save data
        
        ; Set ACK
        ld      a,ACK_BIT
        out     (STATUS_PORT),a
        
        ; Wait for DATA_READY to clear
        ld      b,200           ; Timeout counter
rb_wait_clear:
        in      a,(STATUS_PORT)
        and     DATA_READY
        jr      z,rb_data_cleared
        
        djnz    rb_wait_clear
        
        ; Timeout
        xor     a
        out     (STATUS_PORT),a ; Clear ACK
        pop     af              ; Discard data
        pop     bc
        ld      a,ERROR_TIMEOUT
        ret
        
rb_data_cleared:
        ; Clear ACK
        xor     a
        out     (STATUS_PORT),a
        
        ; Return data
        pop     af
        pop     bc
        ret

; Data section
welcome_msg:    db  'DISKMGR - Exidy Sorcerer Disk Image Manager v1.0', 13, 10
                db  '(c) 2025 Virtual Disk Systems', 13, 10, 13, 10, '$'

menu_msg:       db  13, 10, 'DISK IMAGE MANAGEMENT', 13, 10
                db  '---------------------', 13, 10
                db  'C - Show disk image catalog', 13, 10
                db  'E - Eject a disk', 13, 10
                db  'I - Insert a disk', 13, 10
                db  'Q - Quit', 13, 10
                db  13, 10, 'Enter choice: $'

invalid_msg:    db  13, 10, 'Invalid choice!', 13, 10, '$'

cls_str:        db  27, '[H', 27, '[J', '$'  ; ANSI clear screen

catalog_header: db  'AVAILABLE DISK IMAGES', 13, 10
                db  '--------------------', 13, 10, '$'

catalog_prompt: db  13, 10, 'Press any key to continue...', '$'

drive_prompt:   db  13, 10, 'Enter drive (A-D): $'

image_prompt:   db  13, 10, 'Enter image number: $'

confirm_msg:    db  13, 10, 'Are you sure? (Y/N) $'

canceled_msg:   db  13, 10, 'Operation canceled.', 13, 10, '$'

eject_success:  db  13, 10, 'Disk ejected successfully.', 13, 10, '$'

insert_confirm_1: db  13, 10, 'Insert disk image "', '$'
insert_confirm_2: db  '" into drive ', '$'
insert_confirm_3: db  '? (Y/N) $'

insert_success: db  13, 10, 'Disk inserted successfully.', 13, 10, '$'

invalid_drive_msg: db 13, 10, 'Invalid drive! Must be A-D.', 13, 10, '$'

invalid_image_msg: db 13, 10, 'Invalid image number!', 13, 10, '$'

no_images_msg:  db  13, 10, 'No disk images available!', 13, 10, '$'

comm_error_msg: db  13, 10, 'Communication error!', 13, 10, '$'

disk_error_msg: db  13, 10, 'Disk operation error! Code: $'

exit_msg:       db  13, 10, 'Exiting to CP/M...', 13, 10, '$'

press_key_msg:  db  13, 10, 'Press any key to continue...', '$'

; Variables
image_count:    dw  0           ; Number of images in catalog
selected_drive: db  0           ; Selected drive for operations
selected_image: db  0           ; Selected image index

; Buffers
number_buffer:  ds  6           ; Buffer for number conversion
filename_buffer: ds 33          ; Buffer for filenames (32 chars + null)
format_buffer:  ds  17          ; Buffer for format name (16 chars + null)
catalog_buffer: ds  8*33        ; Buffer for catalog (up to 8 entries)

        end     start
```

## 4. Implementation Details

Let's explain how this works:

1. **Server-Side Image Directory**:
   - The server scans a directory for disk images
   - It maintains information about each image (name, format, size)
   - When disks are ejected/inserted, files are closed/opened

2. **CP/M Program Interface**:
   - The user runs the DISKMGR.COM program under CP/M
   - The menu provides options to:
     - View the disk catalog
     - Eject a disk from a drive
     - Insert a new disk into a drive
   
3. **Hot-Swapping Process**:
   - User selects a drive to eject/insert
   - For insertion, user browses the catalog and selects an image
   - The server handles the file operations
   - CP/M continues to work with the new disk image

## 5. Compilation and Usage

1. **Build the CP/M Program**:
   ```bash
   # Using Z80 assembler (e.g., z80asm)
   z80asm -o diskmgr.com diskmgr.asm
   
   # Copy to a CP/M disk image
   cpmcp -f exidyfds system.img diskmgr.com 0:diskmgr.com
   ```

2. **Configure the Server**:
   ```python
   # Add to server initialization
   self.images_directory = args.images_dir  # Directory of disk images
   ```

3. **Usage on the Sorcerer**:
   - Boot CP/M from the system disk
   - Run the disk manager with: `A>DISKMGR`
   - Follow the on-screen menu to browse and swap disk images

## Advantages

1. **Convenience**: No need to restart the server to change disks
2. **Organization**: Better management of large disk image collections
3. **Flexibility**: Can switch between different CP/M applications without rebooting
4. **Authenticity**: Similar to swapping floppy disks on the original hardware

This enhancement makes the Exidy Sorcerer CP/M system much more usable while maintaining the vintage computing experience.
