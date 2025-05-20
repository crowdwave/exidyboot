#!/usr/bin/env python3
"""
Enhanced CP/M System for Exidy Sorcerer
Features added:
- Support for multiple CP/M disk formats
- Enhanced error handling and recovery
- Based on comprehensive Exidy Sorcerer technical research
"""

import time
import sys
import argparse
import os
import struct
import logging
import json
import hashlib
from threading import Thread, Lock
import wave
import numpy as np
import pyaudio
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple, Union

try:
    import parallel
except ImportError:
    print("Error: pyparallel library not found. Install with 'pip install pyparallel'")
    print("Note: On modern systems, you may need additional drivers for parallel port access")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("cpm_server.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Constants - Port addresses and control bits
DATA_PORT = 0xFF           # Parallel port data register
STATUS_PORT = 0xFE         # Parallel port status register
DATA_READY_BIT = 0x80      # Bit 7: Data Ready flag (device -> Sorcerer)
BUSY_BIT = 0x40            # Bit 6: Busy flag (device -> Sorcerer)
ACK_BIT = 0x01             # Bit 0: Acknowledge flag (Sorcerer -> device)

# Commands
CMD_READ = 0x01            # Read sector
CMD_WRITE = 0x02           # Write sector  
CMD_INIT = 0x03            # Initialize disk system
CMD_STATUS = 0x04          # Get disk status
CMD_BOOTSTRAP = 0x05       # Bootstrap CP/M system
CMD_FORMAT_INFO = 0x06     # Get disk format information
CMD_SET_FORMAT = 0x07      # Set disk format
CMD_VERIFY = 0x08          # Verify sector (checksum)

# Error codes
ERROR_NONE = 0x00          # No error
ERROR_INVALID_DISK = 0x01  # Invalid disk number
ERROR_INVALID_TRACK = 0x02 # Invalid track number
ERROR_INVALID_SECTOR = 0x03 # Invalid sector number
ERROR_READ_FAIL = 0x04     # Read failure
ERROR_WRITE_FAIL = 0x05    # Write failure
ERROR_CRC = 0x06           # CRC/checksum error
ERROR_TIMEOUT = 0x07       # Communication timeout
ERROR_FORMAT = 0x08        # Format error
ERROR_DISK_FULL = 0x09     # Disk full error
ERROR_FILE_NOT_FOUND = 0x0A # File not found
ERROR_UNKNOWN = 0xFF       # Unknown error

# Audio constants
SAMPLE_RATE = 44100        # Samples per second
BAUD_RATE = 1200           # Baud rate for audio encoding

# Communication parameters
TIMEOUT_MS = 5000          # Communication timeout in milliseconds
MAX_RETRIES = 3            # Maximum number of retries for failed operations

# BASIC bootloader program - this is the program that loads CPM
BASIC_BOOTLOADER = """
10 REM PARALLEL PORT BOOTLOADER FOR EXIDY SORCERER
20 START = 60000 : DATA_PORT = 255 : STATUS_PORT = 254
30 LOAD_ADDR = 256 
40 DATA 243,49,0,240,175,211,254,205
50 DATA 45,234,79,205,45,234,71,33
60 DATA 0,1,120,177,40,15,205,45
70 DATA 234,119,35,11,24,245,195,0
80 DATA 1,219,254,230,128,40,249,219
90 DATA 255,245,62,1,211,254,219,254
100 DATA 230,128,32,249,175,211,254,241
110 DATA 201
120 REM POKE BOOTLOADER INTO MEMORY
130 AD = START
140 READ B
150 POKE AD, B
160 AD = AD + 1
170 IF AD < START+47 THEN 140
180 PRINT "LOADING CP/M VIA PARALLEL PORT..."
190 X=USR(START)
200 END
"""

# Disk format definitions
@dataclass
class DiskFormat:
    """Disk format parameters"""
    name: str
    sector_size: int
    sectors_per_track: int
    tracks_per_disk: int
    sides: int
    block_size: int
    dir_entries: int
    reserved_tracks: int
    skew_table: List[int] = None
    description: str = ""

# Standard disk formats for the Exidy Sorcerer
DISK_FORMATS = {
    "exidy_fds": DiskFormat(
        name="Exidy FDS 5.25\" SS DD",
        sector_size=128,
        sectors_per_track=32,
        tracks_per_disk=77,
        sides=1,
        block_size=2048,
        dir_entries=128,
        reserved_tracks=2,
        skew_table=[10,11,20,21,30,31,8,9,18,19,28,29,6,7,16,17,26,27,4,5,14,15,24,25,2,3,12,13,22,23,0,1],
        description="Standard Exidy FDS (Floppy Disk Subsystem) format"
    ),
    "micropolis": DiskFormat(
        name="Micropolis Hard-Sectored",
        sector_size=256,
        sectors_per_track=16,
        tracks_per_disk=77,
        sides=1,
        block_size=2048,
        dir_entries=128,
        reserved_tracks=2,
        description="Micropolis hard-sectored disk format"
    ),
    "dreamdisk": DiskFormat(
        name="Dreamdisk SS 77-track",
        sector_size=256,
        sectors_per_track=16,
        tracks_per_disk=77,
        sides=1,
        block_size=2048,
        dir_entries=128,
        reserved_tracks=2,
        description="Australian Dreamdisk controller format"
    ),
    "ccs_2422": DiskFormat(
        name="CCS 2422 8\" SSSD",
        sector_size=128,
        sectors_per_track=26,
        tracks_per_disk=77,
        sides=1,
        block_size=1024,
        dir_entries=64,
        reserved_tracks=2,
        description="CCS 2422 S-100 Controller 8\" SSSD format"
    )
}

class AudioBootloader(Thread):
    def __init__(self):
        """Initialize the audio bootloader thread."""
        Thread.__init__(self, daemon=True)
        self.running = True
        self.audio = None
        self._prepare_audio()
    
    def _prepare_audio(self):
        """Convert the BASIC bootloader to 1200 baud Kansas City Standard audio."""
        logger.info("Preparing BASIC bootloader audio...")
        
        # Initialize PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Generate audio for leader tone (consistent frequency for synchronization)
        leader_duration = 3.0  # seconds
        leader_samples = self._generate_tone(1200, leader_duration)
        
        # Convert BASIC program to Kansas City Standard audio
        basic_bytes = self._convert_basic_to_bytes()
        bootloader_samples = self._encode_kcs(basic_bytes)
        
        # Trailer tone for end of file
        trailer_samples = self._generate_tone(1200, 1.0)
        
        # Combine leader, bootloader, and trailer
        self.audio_data = np.concatenate([leader_samples, bootloader_samples, trailer_samples])
        
        # Convert to proper format for PyAudio
        self.audio_bytes = (self.audio_data * 32767).astype(np.int16).tobytes()
        
        logger.info(f"BASIC bootloader audio prepared ({len(self.audio_data)/SAMPLE_RATE:.2f}s)")
    
    def _convert_basic_to_bytes(self):
        """Convert the BASIC program to Exidy Sorcerer BASIC format.
        
        Returns:
            Bytes object containing the tokenized BASIC program
        """
        # For this example, we'll use a simplified tokenization
        # In reality, you'd need to tokenize according to the Exidy's BASIC format
        # This is a placeholder that creates a basic "file" structure
        
        # Filename (8 characters, padded with spaces)
        filename = b'BOOTLOAD'
        filename = filename.ljust(8, b' ')
        
        # File type (0x01 for BASIC program)
        file_type = bytes([0x01])
        
        # BASIC program as ASCII with CR line terminators
        program_text = BASIC_BOOTLOADER.strip().replace('\n', '\r')
        program_bytes = program_text.encode('ascii')
        
        # Calculate length (16-bit, little endian)
        length = len(program_bytes)
        length_bytes = struct.pack("<H", length)
        
        # Header + program
        data = filename + file_type + length_bytes + program_bytes
        
        return data
    
    def _generate_tone(self, frequency, duration):
        """Generate a sine wave at the specified frequency and duration.
        
        Args:
            frequency: Frequency in Hz
            duration: Duration in seconds
            
        Returns:
            NumPy array of audio samples
        """
        t = np.linspace(0, duration, int(SAMPLE_RATE * duration), False)
        return np.sin(frequency * 2 * np.pi * t)
    
    def _encode_kcs(self, data_bytes):
        """Encode data using Kansas City Standard (KCS) at 1200 baud.
        
        Args:
            data_bytes: Bytes to encode
            
        Returns:
            NumPy array of audio samples
        """
        # KCS uses 1 cycle at 1200Hz for '0' bit and 2 cycles at 2400Hz for '1' bit
        bit_duration = 1.0 / BAUD_RATE
        samples_per_bit = int(SAMPLE_RATE * bit_duration)
        
        # Start with a leader sequence (SYN characters - 0x16)
        leader_bytes = bytes([0x16, 0x16, 0x16, 0x16])
        
        # Add SOH character
        soh_byte = bytes([0x01])
        
        # Full data to encode
        full_data = leader_bytes + soh_byte + data_bytes
        
        # Prepare audio buffer
        audio_samples = np.array([])
        
        # For each byte in the data
        for byte in full_data:
            # Start bit (always 0)
            tone = self._generate_tone(1200, bit_duration)
            audio_samples = np.concatenate([audio_samples, tone])
            
            # 8 data bits, LSB first
            for i in range(8):
                bit = (byte >> i) & 1
                if bit:
                    # '1' bit = 2 cycles at 2400Hz
                    tone = self._generate_tone(2400, bit_duration)
                else:
                    # '0' bit = 1 cycle at 1200Hz
                    tone = self._generate_tone(1200, bit_duration)
                audio_samples = np.concatenate([audio_samples, tone])
            
            # Stop bits (always 1, send 2 stop bits)
            for _ in range(2):
                tone = self._generate_tone(2400, bit_duration)
                audio_samples = np.concatenate([audio_samples, tone])
        
        # Add EOT (End of Transmission) marker
        eot_byte = bytes([0x04])
        for byte in eot_byte:
            # Start bit
            tone = self._generate_tone(1200, bit_duration)
            audio_samples = np.concatenate([audio_samples, tone])
            
            # 8 data bits
            for i in range(8):
                bit = (byte >> i) & 1
                if bit:
                    tone = self._generate_tone(2400, bit_duration)
                else:
                    tone = self._generate_tone(1200, bit_duration)
                audio_samples = np.concatenate([audio_samples, tone])
            
            # Stop bits
            for _ in range(2):
                tone = self._generate_tone(2400, bit_duration)
                audio_samples = np.concatenate([audio_samples, tone])
        
        return audio_samples
    
    def run(self):
        """Run the audio playback in a loop."""
        logger.info("Starting audio bootloader loop...")
        
        # Open a stream
        stream = self.audio.open(
            format=self.audio.get_format_from_width(2),
            channels=1,
            rate=SAMPLE_RATE,
            output=True
        )
        
        try:
            # Loop the audio continuously
            while self.running:
                stream.write(self.audio_bytes)
                # Add a short pause between repetitions
                time.sleep(0.5)
        finally:
            # Clean up
            stream.stop_stream()
            stream.close()
            self.audio.terminate()
    
    def stop(self):
        """Stop the audio playback."""
        self.running = False

class CPMDiskServer:
    def __init__(self, port_address=0x378, disk_images=None, cpm_loader_path=None, disk_format="exidy_fds"):
        """Initialize the CP/M disk server.
        
        Args:
            port_address: Base address of the parallel port
            disk_images: List of paths to disk image files (at least 2)
            cpm_loader_path: Path to the CP/M system loader binary
            disk_format: Default disk format to use
        """
        # Initialize parallel port
        try:
            self.port = parallel.Parallel(port_address)
            logger.info(f"Parallel port initialized at 0x{port_address:X}")
        except Exception as e:
            logger.error(f"Error initializing parallel port: {e}")
            sys.exit(1)
        
        # Set disk format
        if disk_format in DISK_FORMATS:
            self.disk_format = DISK_FORMATS[disk_format]
            logger.info(f"Using disk format: {self.disk_format.name}")
        else:
            logger.error(f"Unknown disk format: {disk_format}")
            logger.info(f"Available formats: {', '.join(DISK_FORMATS.keys())}")
            sys.exit(1)
        
        # Load CP/M bootstrap loader if provided
        self.cpm_loader = None
        if cpm_loader_path and os.path.exists(cpm_loader_path):
            try:
                with open(cpm_loader_path, "rb") as f:
                    self.cpm_loader = f.read()
                logger.info(f"Loaded CP/M bootstrap loader ({len(self.cpm_loader)} bytes)")
            except Exception as e:
                logger.error(f"Error loading CP/M loader: {e}")
                sys.exit(1)
        else:
            # If no specific loader provided, use the default CP/M system tracks
            logger.info("No specific CP/M loader provided, will use system tracks")
        
        # Load disk images
        self.disk_images = []
        self.disk_files = []
        self.disk_formats = []  # Format for each disk
        
        if disk_images and len(disk_images) > 0:
            for i, image_path in enumerate(disk_images[:4]):  # Support up to 4 disks
                try:
                    if not os.path.exists(image_path):
                        # Create a new empty disk image
                        logger.info(f"Creating new disk image: {image_path}")
                        self._create_empty_disk(image_path)
                    
                    # Open the disk image file
                    disk_file = open(image_path, "r+b")
                    self.disk_files.append(disk_file)
                    self.disk_images.append(image_path)
                    self.disk_formats.append(self.disk_format)  # Default format
                    
                    # Try to detect format if not first disk (system disk)
                    if i > 0:
                        detected_format = self._detect_disk_format(disk_file)
                        if detected_format:
                            self.disk_formats[i] = detected_format
                            logger.info(f"Detected disk format for {image_path}: {detected_format.name}")
                    
                    logger.info(f"Loaded disk image {image_path} as drive {chr(65+i)}: using format {self.disk_formats[i].name}")
                except Exception as e:
                    logger.error(f"Error loading disk image {image_path}: {e}")
                    sys.exit(1)
        
        if not self.disk_images or len(self.disk_images) < 2:
            logger.error("At least two disk images are required.")
            sys.exit(1)
        
        # Initialize communication lock
        self.comm_lock = Lock()
        
        # Cache the bootstrap code from disk image if no loader provided
        if not self.cpm_loader:
            self._cache_system_tracks()
        
        # Initialize statistics
        self.stats = {
            "read_operations": 0,
            "write_operations": 0,
            "errors": 0,
            "retries": 0,
            "start_time": time.time()
        }
        
        # Start the audio bootloader thread
        self.audio_thread = AudioBootloader()
        self.audio_thread.start()
        
        logger.info("CP/M Disk Server initialized and ready")
    
    def _detect_disk_format(self, disk_file):
        """Try to detect the disk format from the disk image.
        
        Args:
            disk_file: Open file object for the disk image
            
        Returns:
            DiskFormat object if detected, None otherwise
        """
        # Save current position
        current_pos = disk_file.tell()
        
        # Try to detect format based on size and content
        try:
            # Get file size
            disk_file.seek(0, os.SEEK_END)
            file_size = disk_file.tell()
            disk_file.seek(0)
            
            # Check against known format sizes
            for fmt_name, fmt in DISK_FORMATS.items():
                expected_size = fmt.sector_size * fmt.sectors_per_track * fmt.tracks_per_disk * fmt.sides
                if abs(file_size - expected_size) < 1024:  # Allow small difference
                    # Size matches, check directory structure
                    dir_offset = fmt.reserved_tracks * fmt.sectors_per_track * fmt.sector_size
                    disk_file.seek(dir_offset)
                    dir_data = disk_file.read(fmt.sector_size)
                    
                    # Check for typical CP/M directory entries
                    # (first byte of unused entries is typically 0xE5)
                    if dir_data[0] in [0xE5, 0x00] or (dir_data[0] >= ord('A') and dir_data[0] <= ord('Z')):
                        return fmt
        except Exception as e:
            logger.debug(f"Error in format detection: {e}")
        finally:
            # Restore position
            disk_file.seek(current_pos)
        
        return None
    
    def _cache_system_tracks(self):
        """Cache the CP/M system tracks from the first disk image."""
        logger.info("Caching CP/M system tracks from disk image")
        
        # Get format of first disk
        fmt = self.disk_formats[0]
        
        # CP/M system typically resides in the first two tracks
        system_size = fmt.reserved_tracks * fmt.sectors_per_track * fmt.sector_size
        
        try:
            # Read the system tracks from the first disk
            self.disk_files[0].seek(0)
            self.cpm_loader = self.disk_files[0].read(system_size)
            logger.info(f"Cached {len(self.cpm_loader)} bytes from system tracks")
        except Exception as e:
            logger.error(f"Error caching system tracks: {e}")
            self.cpm_loader = None
    
    def _create_empty_disk(self, path):
        """Create a new empty CP/M disk image file."""
        # Use current disk format
        fmt = self.disk_format
        disk_size = fmt.sector_size * fmt.sectors_per_track * fmt.tracks_per_disk * fmt.sides
        
        with open(path, "wb") as f:
            # Write a completely empty disk
            f.write(b'\xE5' * disk_size)  # 0xE5 is CP/M's empty directory marker
    
    def _calculate_offset(self, disk, track, sector):
        """Calculate the file offset for a given disk sector.
        
        Args:
            disk: Disk number
            track: Track number
            sector: Sector number
            
        Returns:
            Offset in bytes from the start of the disk image
        """
        # Get format for this disk
        fmt = self.disk_formats[disk]
        
        # Apply skew table if available
        if fmt.skew_table and sector < len(fmt.skew_table):
            physical_sector = fmt.skew_table[sector]
        else:
            physical_sector = sector
        
        # CP/M sectors are usually 1-based, but our skew table is 0-based
        sector_index = physical_sector if sector >= 0 else sector
        
        offset = track * fmt.sectors_per_track * fmt.sector_size + sector_index * fmt.sector_size
        return offset
    
    def _calculate_checksum(self, data):
        """Calculate checksum for data verification.
        
        Args:
            data: Bytes to checksum
            
        Returns:
            16-bit checksum value
        """
        # Simple 16-bit checksum
        checksum = 0
        for b in data:
            checksum = (checksum + b) & 0xFFFF
        return checksum
    
    def receive_command(self):
        """Wait for and receive a command from the Z80.
        
        According to Exidy Sorcerer technical documentation:
        - The Sorcerer sets Data Available bit when ready to send a command
        - We wait for it, then read the data byte
        - We acknowledge with ACK_BIT
        - We wait for Sorcerer to clear Data Available
        - We clear ACK_BIT
        
        Returns:
            Command byte or None if timeout
        """
        with self.comm_lock:
            # Wait for data ready with timeout
            start_time = time.time()
            while not (self.port.getInSelected() & DATA_READY_BIT):
                if (time.time() - start_time) * 1000 > TIMEOUT_MS:
                    logger.warning("Timeout waiting for command")
                    return None
                time.sleep(0.001)
            
            # Read the command
            command = self.port.getData()
            
            # Acknowledge receipt with ACK_BIT
            self.port.setDataStrobe(ACK_BIT)
            
            # Wait for Sorcerer to clear DATA_READY_BIT
            start_time = time.time()
            while self.port.getInSelected() & DATA_READY_BIT:
                if (time.time() - start_time) * 1000 > TIMEOUT_MS:
                    logger.warning("Timeout waiting for DATA_READY_BIT to clear")
                    self.port.setDataStrobe(0)  # Clear ACK in case of timeout
                    return None
                time.sleep(0.001)
            
            # Clear ACK_BIT
            self.port.setDataStrobe(0)
            
            return command
    
    def receive_byte(self):
        """Receive a byte from the Z80.
        
        Returns:
            Data byte or None if timeout
        """
        with self.comm_lock:
            # Wait for DATA_READY_BIT to be set
            start_time = time.time()
            while not (self.port.getInSelected() & DATA_READY_BIT):
                if (time.time() - start_time) * 1000 > TIMEOUT_MS:
                    logger.warning("Timeout waiting for data")
                    return None
                time.sleep(0.001)
            
            # Read the data byte
            data = self.port.getData()
            
            # Acknowledge receipt by setting ACK_BIT
            self.port.setDataStrobe(ACK_BIT)
            
            # Wait for Sorcerer to clear DATA_READY_BIT
            start_time = time.time()
            while self.port.getInSelected() & DATA_READY_BIT:
                if (time.time() - start_time) * 1000 > TIMEOUT_MS:
                    logger.warning("Timeout waiting for DATA_READY_BIT to clear")
                    self.port.setDataStrobe(0)  # Clear ACK in case of timeout
                    return None
                time.sleep(0.001)
            
            # Clear acknowledge
            self.port.setDataStrobe(0)
            
            return data
    
    def send_byte(self, data_byte):
        """Send a byte to the Z80.
        
        Args:
            data_byte: Byte to send
            
        Returns:
            True if successful, False on timeout
        """
        with self.comm_lock:
            # Check if device is busy
            start_time = time.time()
            while self.port.getInSelected() & BUSY_BIT:
                if (time.time() - start_time) * 1000 > TIMEOUT_MS:
                    logger.warning("Timeout waiting for BUSY_BIT to clear")
                    return False
                time.sleep(0.001)
            
            # Set the data byte
            self.port.setData(data_byte)
            
            # Signal data ready
            self.port.setDataStrobe(DATA_READY_BIT)
            
            # Wait for Sorcerer to acknowledge (set ACK_BIT)
            start_time = time.time()
            while not (self.port.getInSelected() & ACK_BIT):
                if (time.time() - start_time) * 1000 > TIMEOUT_MS:
                    logger.warning("Timeout waiting for ACK_BIT")
                    self.port.setDataStrobe(0)  # Clear DATA_READY in case of timeout
                    return False
                time.sleep(0.001)
            
            # Clear data ready
            self.port.setDataStrobe(0)
            
            # Wait for Sorcerer to clear acknowledge
            start_time = time.time()
            while self.port.getInSelected() & ACK_BIT:
                if (time.time() - start_time) * 1000 > TIMEOUT_MS:
                    logger.warning("Timeout waiting for ACK_BIT to clear")
                    return False
                time.sleep(0.001)
            
            return True
    
    def handle_bootstrap(self):
        """Handle CP/M bootstrap loading."""
        logger.info("Handling CP/M bootstrap request")
        
        if not self.cpm_loader:
            logger.error("No CP/M loader available")
            self.send_byte(ERROR_UNKNOWN)  # Error
            return
        
        try:
            # Send success status
            self.send_byte(ERROR_NONE)
            
            # Send the size of the loader (16-bit little endian)
            size = len(self.cpm_loader)
            self.send_byte(size & 0xFF)         # Low byte
            self.send_byte((size >> 8) & 0xFF)  # High byte
            
            # Send the loader data
            for b in self.cpm_loader:
                if not self.send_byte(b):
                    logger.error("Failed to send bootstrap data")
                    return
            
            # Send checksum for verification (16-bit)
            checksum = self._calculate_checksum(self.cpm_loader)
            self.send_byte(checksum & 0xFF)      # Low byte
            self.send_byte((checksum >> 8) & 0xFF) # High byte
            
            logger.info(f"Sent {size} bytes of CP/M bootstrap loader")
            
        except Exception as e:
            logger.error(f"Error sending bootstrap loader: {e}")
            self.send_byte(ERROR_UNKNOWN)  # Error
    
    def handle_read_sector(self):
        """Handle a read sector command."""
        # Receive parameters
        disk = self.receive_byte()
        track = self.receive_byte()
        sector = self.receive_byte()
        
        if disk is None or track is None or sector is None:
            logger.error("Timeout receiving read sector parameters")
            return
        
        logger.debug(f"READ: Disk={disk}, Track={track}, Sector={sector}")
        self.stats["read_operations"] += 1
        
        # Check if disk number is valid
        if disk >= len(self.disk_files):
            logger.error(f"Invalid disk number: {disk}")
            self.send_byte(ERROR_INVALID_DISK)  # Error
            self.stats["errors"] += 1
            return
        
        # Implement retry logic
        for retry in range(MAX_RETRIES):
            try:
                # Calculate file offset
                offset = self._calculate_offset(disk, track, sector)
                
                # Seek to position
                self.disk_files[disk].seek(offset)
                
                # Read the sector data
                data = self.disk_files[disk].read(self.disk_formats[disk].sector_size)
                
                # If we got less than a full sector, pad with 0xE5
                if len(data) < self.disk_formats[disk].sector_size:
                    data = data + bytes([0xE5] * (self.disk_formats[disk].sector_size - len(data)))
                
                # Calculate checksum
                checksum = self._calculate_checksum(data)
                
                # Send success status
                if not self.send_byte(ERROR_NONE):
                    logger.error("Failed to send status")
                    if retry < MAX_RETRIES - 1:
                        self.stats["retries"] += 1
                        continue
                    else:
                        self.stats["errors"] += 1
                        return
                
                # Send the sector data
                for b in data:
                    if not self.send_byte(b):
                        logger.error("Failed to send sector data")
                        if retry < MAX_RETRIES - 1:
                            self.stats["retries"] += 1
                            break
                        else:
                            self.stats["errors"] += 1
                            return
                else:
                    # Send checksum (16-bit)
                    self.send_byte(checksum & 0xFF)      # Low byte
                    self.send_byte((checksum >> 8) & 0xFF) # High byte
                    
                    logger.debug(f"Read sector successful")
                    return  # Success
                
                # If we get here, the inner loop was broken (send error)
                continue
                
            except Exception as e:
                logger.error(f"Error reading sector: {e}")
                if retry < MAX_RETRIES - 1:
                    logger.info(f"Retrying read operation (attempt {retry+1}/{MAX_RETRIES})")
                    self.stats["retries"] += 1
                    continue
                else:
                    self.send_byte(ERROR_READ_FAIL)  # Error
                    self.stats["errors"] += 1
                    return
        
        # If we get here, all retries failed
        self.send_byte(ERROR_READ_FAIL)
        self.stats["errors"] += 1
    
    def handle_write_sector(self):
        """Handle a write sector command."""
        # Receive parameters
        disk = self.receive_byte()
        track = self.receive_byte()
        sector = self.receive_byte()
        write_type = self.receive_byte()  # 0=normal, 1=directory
        
        if disk is None or track is None or sector is None or write_type is None:
            logger.error("Timeout receiving write sector parameters")
            return
        
        logger.debug(f"WRITE: Disk={disk}, Track={track}, Sector={sector}, Type={write_type}")
        self.stats["write_operations"] += 1
        
        # Check if disk number is valid
        if disk >= len(self.disk_files):
            logger.error(f"Invalid disk number: {disk}")
            self.send_byte(ERROR_INVALID_DISK)  # Error
            self.stats["errors"] += 1
            return
        
        # Implement retry logic
        for retry in range(MAX_RETRIES):
            try:
                # Calculate file offset
                offset = self._calculate_offset(disk, track, sector)
                
                # Receive the sector data
                data = bytearray()
                for _ in range(self.disk_formats[disk].sector_size):
                    b = self.receive_byte()
                    if b is None:
                        logger.error("Timeout receiving sector data")
                        if retry < MAX_RETRIES - 1:
                            self.stats["retries"] += 1
                            break
                        else:
                            self.stats["errors"] += 1
                            return
                    data.append(b)
                else:
                    # Receive checksum (16-bit)
                    checksum_low = self.receive_byte()
                    checksum_high = self.receive_byte()
                    
                    if checksum_low is None or checksum_high is None:
                        logger.error("Timeout receiving checksum")
                        if retry < MAX_RETRIES - 1:
                            self.stats["retries"] += 1
                            continue
                        else:
                            self.stats["errors"] += 1
                            return
                    
                    # Verify checksum
                    received_checksum = (checksum_high << 8) | checksum_low
                    calculated_checksum = self._calculate_checksum(data)
                    
                    if received_checksum != calculated_checksum:
                        logger.error(f"Checksum mismatch: received {received_checksum:04X}, calculated {calculated_checksum:04X}")
                        self.send_byte(ERROR_CRC)  # Checksum error
                        if retry < MAX_RETRIES - 1:
                            self.stats["retries"] += 1
                            continue
                        else:
                            self.stats["errors"] += 1
                            return
                    
                    # Seek to position
                    self.disk_files[disk].seek(offset)
                    
                    # Write the sector data
                    self.disk_files[disk].write(data)
                    self.disk_files[disk].flush()
                    
                    # Send success status
                    self.send_byte(ERROR_NONE)
                    
                    logger.debug(f"Write sector successful")
                    return  # Success
                
                # If we get here, the inner loop was broken (receive error)
                continue
                
            except Exception as e:
                logger.error(f"Error writing sector: {e}")
                if retry < MAX_RETRIES - 1:
                    logger.info(f"Retrying write operation (attempt {retry+1}/{MAX_RETRIES})")
                    self.stats["retries"] += 1
                    continue
                else:
                    self.send_byte(ERROR_WRITE_FAIL)  # Error
                    self.stats["errors"] += 1
                    return
        
        # If we get here, all retries failed
        self.send_byte(ERROR_WRITE_FAIL)
        self.stats["errors"] += 1
    
    def handle_init(self):
        """Handle disk initialization command."""
        logger.info("Initializing disk system")
        
        # Return success
        self.send_byte(ERROR_NONE)
    
    def handle_status(self):
        """Handle get disk status command."""
        # Just return ready status for now
        self.send_byte(ERROR_NONE)
    
    def handle_format_info(self):
        """Handle get disk format information command."""
        # Receive disk number
        disk = self.receive_byte()
        
        if disk is None:
            logger.error("Timeout receiving disk number")
            return
        
        # Check if disk number is valid
        if disk >= len(self.disk_files):
            logger.error(f"Invalid disk number: {disk}")
            self.send_byte(ERROR_INVALID_DISK)  # Error
            return
        
        # Get format for this disk
        fmt = self.disk_formats[disk]
        
        # Send success status
        self.send_byte(ERROR_NONE)
        
        # Send format information
        self.send_byte(fmt.sector_size & 0xFF)         # Low byte of sector size
        self.send_byte((fmt.sector_size >> 8) & 0xFF)  # High byte of sector size
        self.send_byte(fmt.sectors_per_track)          # Sectors per track
        self.send_byte(fmt.tracks_per_disk)            # Tracks per disk
        self.send_byte(fmt.sides)                      # Sides
        self.send_byte(fmt.reserved_tracks)            # Reserved tracks
        
        logger.debug(f"Sent format info for disk {disk}")
    
    def handle_set_format(self):
        """Handle set disk format command."""
        # Receive disk number
        disk = self.receive_byte()
        
        if disk is None:
            logger.error("Timeout receiving disk number")
            return
        
        # Receive format index
        format_index = self.receive_byte()
        
        if format_index is None:
            logger.error("Timeout receiving format index")
            return
        
        # Check if disk number is valid
        if disk >= len(self.disk_files):
            logger.error(f"Invalid disk number: {disk}")
            self.send_byte(ERROR_INVALID_DISK)  # Error
            return
        
        # Check if format index is valid
        format_keys = list(DISK_FORMATS.keys())
        if format_index >= len(format_keys):
            logger.error(f"Invalid format index: {format_index}")
            self.send_byte(ERROR_FORMAT)  # Format error
            return
        
        # Set format for this disk
        self.disk_formats[disk] = DISK_FORMATS[format_keys[format_index]]
        
        # Send success status
        self.send_byte(ERROR_NONE)
        
        logger.info(f"Set format for disk {disk} to {self.disk_formats[disk].name}")
    
    def handle_verify(self):
        """Handle verify sector command."""
        # Receive parameters
        disk = self.receive_byte()
        track = self.receive_byte()
        sector = self.receive_byte()
        
        if disk is None or track is None or sector is None:
            logger.error("Timeout receiving verify sector parameters")
            return
        
        logger.debug(f"VERIFY: Disk={disk}, Track={track}, Sector={sector}")
        
        # Check if disk number is valid
        if disk >= len(self.disk_files):
            logger.error(f"Invalid disk number: {disk}")
            self.send_byte(ERROR_INVALID_DISK)  # Error
            return
        
        try:
            # Calculate file offset
            offset = self._calculate_offset(disk, track, sector)
            
            # Seek to position
            self.disk_files[disk].seek(offset)
            
            # Read the sector data
            data = self.disk_files[disk].read(self.disk_formats[disk].sector_size)
            
            # If we got less than a full sector, pad with 0xE5
            if len(data) < self.disk_formats[disk].sector_size:
                data = data + bytes([0xE5] * (self.disk_formats[disk].sector_size - len(data)))
            
            # Calculate checksum
            checksum = self._calculate_checksum(data)
            
            # Send success status
            self.send_byte(ERROR_NONE)
            
            # Send checksum (16-bit)
            self.send_byte(checksum & 0xFF)      # Low byte
            self.send_byte((checksum >> 8) & 0xFF) # High byte
            
            logger.debug(f"Verify sector successful")
            
        except Exception as e:
            logger.error(f"Error verifying sector: {e}")
            self.send_byte(ERROR_READ_FAIL)  # Error
    
    def print_stats(self):
        """Print server statistics."""
        elapsed = time.time() - self.stats["start_time"]
        reads_per_sec = self.stats["read_operations"] / elapsed if elapsed > 0 else 0
        writes_per_sec = self.stats["write_operations"] / elapsed if elapsed > 0 else 0
        
        print("\nServer Statistics:")
        print(f"  Running time: {elapsed:.1f} seconds")
        print(f"  Read operations: {self.stats['read_operations']} ({reads_per_sec:.2f}/s)")
        print(f"  Write operations: {self.stats['write_operations']} ({writes_per_sec:.2f}/s)")
        print(f"  Errors: {self.stats['errors']}")
        print(f"  Retries: {self.stats['retries']}")
    
    def run(self):
        """Main server loop."""
        logger.info("CP/M Disk Server running. Press Ctrl+C to quit.")
        
        try:
            while True:
                # Wait for a command
                command = self.receive_command()
                
                # Handle timeout
                if command is None:
                    continue
                
                # Handle the command
                if command == CMD_READ:
                    self.handle_read_sector()
                elif command == CMD_WRITE:
                    self.handle_write_sector()
                elif command == CMD_INIT:
                    self.handle_init()
                elif command == CMD_STATUS:
                    self.handle_status()
                elif command == CMD_BOOTSTRAP:
                    self.handle_bootstrap()
                elif command == CMD_FORMAT_INFO:
                    self.handle_format_info()
                elif command == CMD_SET_FORMAT:
                    self.handle_set_format()
                elif command == CMD_VERIFY:
                    self.handle_verify()
                else:
                    logger.warning(f"Unknown command: 0x{command:02X}")
                    self.send_byte(ERROR_UNKNOWN)  # Unknown command
                
        except KeyboardInterrupt:
            logger.info("Server stopped by user")
            self.print_stats()
        finally:
            # Stop audio thread
            self.audio_thread.stop()
            
            # Close disk files
            for file in self.disk_files:
                file.close()
            
            logger.info("Server shutdown complete")

def save_bootloader_audio(filename):
    """Save the bootloader audio to a WAV file.
    
    Args:
        filename: Path to save the WAV file
    """
    # Create a temporary AudioBootloader to generate the audio
    bootloader = AudioBootloader()
    
    # Save as WAV file
    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(bootloader.audio_bytes)
    
    logger.info(f"Bootloader audio saved to {filename}")

def main():
    parser = argparse.ArgumentParser(description='Enhanced CP/M Disk Server for Exidy Sorcerer')
    parser.add_argument('--port', type=str, default='0x378',
                        help='Parallel port address (default: 0x378)')
    parser.add_argument('--disks', type=str, nargs='+', required=True,
                        help='Paths to disk image files (at least 2 required)')
    parser.add_argument('--loader', type=str,
                        help='Path to custom CP/M bootstrap loader binary')
    parser.add_argument('--format', type=str, default='exidy_fds',
                        help=f'Disk format to use (default: exidy_fds). Available formats: {", ".join(DISK_FORMATS.keys())}')
    parser.add_argument('--save-audio', type=str, metavar='FILENAME',
                        help='Save bootloader audio to a WAV file')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable verbose logging')
    parser.add_argument('--list-formats', action='store_true',
                        help='List available disk formats and exit')
    
    args = parser.parse_args()
    
    # Set logging level
    if args.verbose:
        logger.setLevel(logging.DEBUG)
    
    # List formats if requested
    if args.list_formats:
        print("Available disk formats:")
        for key, fmt in DISK_FORMATS.items():
            print(f"  {key}: {fmt.name}")
            print(f"    Sector size: {fmt.sector_size} bytes")
            print(f"    Sectors per track: {fmt.sectors_per_track}")
            print(f"    Tracks: {fmt.tracks_per_disk}")
            print(f"    Sides: {fmt.sides}")
            print(f"    Block size: {fmt.block_size} bytes")
            print(f"    Directory entries: {fmt.dir_entries}")
            print(f"    Reserved tracks: {fmt.reserved_tracks}")
            print(f"    Description: {fmt.description}")
            print()
        return
    
    # If requested, save bootloader audio to a file and exit
    if args.save_audio:
        save_bootloader_audio(args.save_audio)
        return
    
    # Parse port address
    try:
        port_address = int(args.port, 0)  # Parse as integer with auto base detection
    except ValueError:
        logger.error(f"Invalid port address: {args.port}")
        sys.exit(1)
    
    # Check disk format
    if args.format not in DISK_FORMATS:
        logger.error(f"Unknown disk format: {args.format}")
        logger.info(f"Available formats: {', '.join(DISK_FORMATS.keys())}")
        sys.exit(1)
    
    # Start the server
    server = CPMDiskServer(port_address, args.disks, args.loader, args.format)
    server.run()

if __name__ == "__main__":
    main()
