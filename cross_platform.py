#!/usr/bin/env python3
"""
Cross-Platform Utilities for Drone Mission Control

Provides OS-agnostic functions for:
- Serial port auto-discovery
- File path handling
- Console operations
- High-precision timing
"""

import os
import sys
import time
import platform
from pathlib import Path
from typing import List, Optional, Tuple

# =============================================================================
# PORT FINDER - Auto-discovery of telemetry radios
# =============================================================================

# Known telemetry radio USB identifiers (VID:PID)
KNOWN_RADIO_IDS = {
    # SiK Radio (common in 3DR, Holybro, mRo)
    (0x0403, 0x6001): "FTDI (SiK Radio)",
    (0x0403, 0x6015): "FTDI (SiK Radio)",
    (0x26AC, 0x0011): "3DR SiK Radio",
    
    # Holybro Telemetry
    (0x10C4, 0xEA60): "Silicon Labs (Holybro)",
    
    # Cube/Here/ProfiCNC
    (0x2DAE, 0x1011): "Cube Orange",
    (0x2DAE, 0x1016): "Cube Black",
    (0x2DAE, 0x1012): "CubeYellow",
    
    # SIYI MK15
    (0x1A86, 0x7523): "CH340 (SIYI/Generic)",
    (0x1A86, 0x55D4): "CH9102 (SIYI MK15)",
    
    # Generic USB-Serial
    (0x067B, 0x2303): "Prolific (Generic)",
    (0x10C4, 0xEA70): "CP2105 (Generic)",
}


class PortFinder:
    """
    Cross-platform serial port auto-discovery for telemetry radios.
    
    Usage:
        finder = PortFinder()
        ports = finder.find_telemetry_ports()
        
        # Or let user select
        port = finder.select_port()
    """
    
    def __init__(self):
        self.os_name = platform.system()  # 'Windows', 'Linux', 'Darwin'
        self._ports_cache = None
    
    def list_all_ports(self) -> List[dict]:
        """
        List all available serial ports with metadata.
        
        Returns:
            List of dicts with keys: device, description, vid, pid, is_telemetry
        """
        try:
            import serial.tools.list_ports
        except ImportError:
            print("ERROR: pyserial not installed. Run: pip install pyserial")
            return []
        
        ports = []
        for port in serial.tools.list_ports.comports():
            vid = port.vid
            pid = port.pid
            
            # Check if it's a known telemetry radio
            is_telemetry = False
            radio_name = None
            if vid and pid:
                key = (vid, pid)
                if key in KNOWN_RADIO_IDS:
                    is_telemetry = True
                    radio_name = KNOWN_RADIO_IDS[key]
            
            ports.append({
                'device': port.device,
                'description': port.description,
                'vid': vid,
                'pid': pid,
                'is_telemetry': is_telemetry,
                'radio_name': radio_name,
                'hwid': port.hwid,
            })
        
        return sorted(ports, key=lambda x: x['device'])
    
    def find_telemetry_ports(self) -> List[str]:
        """
        Find ports that are likely telemetry radios.
        
        Returns:
            List of device strings (e.g., '/dev/ttyUSB0' or 'COM3')
        """
        all_ports = self.list_all_ports()
        telemetry_ports = [p['device'] for p in all_ports if p['is_telemetry']]
        
        # If no known radios found, return all serial ports
        if not telemetry_ports:
            # Filter to likely candidates on Linux
            if self.os_name == 'Linux':
                return [p['device'] for p in all_ports 
                       if 'ttyUSB' in p['device'] or 'ttyACM' in p['device']]
            else:
                return [p['device'] for p in all_ports]
        
        return telemetry_ports
    
    def select_port(self, prompt: str = "Select telemetry radio") -> Optional[str]:
        """
        Interactive port selection.
        
        Args:
            prompt: Message to display
            
        Returns:
            Selected port device string, or None if cancelled
        """
        ports = self.list_all_ports()
        
        if not ports:
            print("No serial ports found!")
            return None
        
        print(f"\n{prompt}")
        print("-" * 50)
        
        for i, port in enumerate(ports, 1):
            marker = "★" if port['is_telemetry'] else " "
            name = port['radio_name'] or port['description']
            print(f"  {marker} [{i}] {port['device']} - {name}")
        
        print("-" * 50)
        print("  ★ = Known telemetry radio")
        
        while True:
            try:
                choice = input("\nEnter number (or 'q' to quit): ").strip()
                if choice.lower() == 'q':
                    return None
                
                idx = int(choice) - 1
                if 0 <= idx < len(ports):
                    return ports[idx]['device']
                print(f"Invalid selection. Enter 1-{len(ports)}")
            except ValueError:
                print("Enter a number")
    
    def get_connection_string(self, device: str, baud: int = 57600) -> str:
        """
        Get pymavlink connection string for device.
        
        Args:
            device: Serial device path
            baud: Baud rate
            
        Returns:
            Connection string for mavutil.mavlink_connection()
        """
        # pymavlink accepts device path directly
        return device


# =============================================================================
# FILE PATH UTILITIES - OS-agnostic path handling
# =============================================================================

def get_project_root() -> Path:
    """Get project root directory."""
    return Path(__file__).parent.resolve()


def get_data_path(*parts) -> Path:
    """
    Get path relative to project data directory.
    
    Args:
        *parts: Path components (e.g., 'output', 'targets.json')
        
    Returns:
        Resolved Path object
    """
    return get_project_root().joinpath(*parts)


def ensure_dir(path: Path) -> Path:
    """Create directory if it doesn't exist."""
    path.mkdir(parents=True, exist_ok=True)
    return path


# =============================================================================
# CONSOLE UTILITIES - OS-agnostic terminal operations
# =============================================================================

def clear_screen():
    """Clear terminal screen (cross-platform)."""
    if os.name == 'nt':  # Windows
        os.system('cls')
    else:  # Linux, macOS
        os.system('clear')


def get_terminal_size() -> Tuple[int, int]:
    """Get terminal size (columns, rows)."""
    try:
        size = os.get_terminal_size()
        return size.columns, size.lines
    except OSError:
        return 80, 24  # Default fallback


# =============================================================================
# HIGH-PRECISION TIMING
# =============================================================================

class PrecisionTimer:
    """
    High-precision timing for control loops.
    
    Uses time.perf_counter() for consistent timing across platforms.
    
    Usage:
        timer = PrecisionTimer(target_hz=10)  # 10 Hz loop
        
        while running:
            timer.tick()
            
            # ... do work ...
            
            timer.wait()  # Sleep remaining time
    """
    
    def __init__(self, target_hz: float = 10.0):
        """
        Initialize timer.
        
        Args:
            target_hz: Target loop frequency in Hz
        """
        self.target_hz = target_hz
        self.target_period = 1.0 / target_hz
        self.last_tick = time.perf_counter()
        self.delta_time = 0.0
        self.actual_hz = 0.0
    
    def tick(self):
        """Mark start of loop iteration."""
        now = time.perf_counter()
        self.delta_time = now - self.last_tick
        if self.delta_time > 0:
            self.actual_hz = 1.0 / self.delta_time
        self.last_tick = now
    
    def wait(self):
        """Sleep for remaining time in period."""
        elapsed = time.perf_counter() - self.last_tick
        remaining = self.target_period - elapsed
        
        if remaining > 0:
            time.sleep(remaining)
    
    def get_delta(self) -> float:
        """Get time since last tick (seconds)."""
        return self.delta_time
    
    def get_hz(self) -> float:
        """Get actual loop frequency."""
        return self.actual_hz


# =============================================================================
# PROCESS GUARD - Multiprocessing safety
# =============================================================================

def is_main_process() -> bool:
    """Check if running in main process (for multiprocessing safety)."""
    import multiprocessing
    return multiprocessing.current_process().name == 'MainProcess'


# =============================================================================
# TEST
# =============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Cross-Platform Utilities Test")
    print("=" * 60)
    
    print(f"\nOS: {platform.system()} ({os.name})")
    print(f"Python: {sys.version}")
    
    # Test port finder
    print("\n--- Port Discovery ---")
    finder = PortFinder()
    ports = finder.list_all_ports()
    
    if ports:
        for p in ports:
            marker = "★" if p['is_telemetry'] else " "
            print(f"  {marker} {p['device']}: {p['description']}")
    else:
        print("  No serial ports found")
    
    # Test paths
    print("\n--- Path Utilities ---")
    print(f"  Project root: {get_project_root()}")
    print(f"  Data path: {get_data_path('output', 'targets.json')}")
    
    # Test timer
    print("\n--- Precision Timer ---")
    timer = PrecisionTimer(target_hz=10)
    for i in range(3):
        timer.tick()
        time.sleep(0.05)  # Simulate work
        timer.wait()
        print(f"  Iteration {i+1}: dt={timer.get_delta():.4f}s, hz={timer.get_hz():.1f}")
    
    print("\n" + "=" * 60)
