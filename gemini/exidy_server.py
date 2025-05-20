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
