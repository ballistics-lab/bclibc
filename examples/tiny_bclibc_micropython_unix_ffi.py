#!/usr/bin/env micropython
"""
BCLIBC - Ballistics library wrapper for MicroPython
"""

import ffi
import sys
import os


def _get_dirname(filepath):
    """Get directory name from a file path (manual implementation)."""
    # Remove trailing slash if present
    if filepath.endswith("/"):
        filepath = filepath[:-1]

    # Find the last slash
    last_slash = filepath.rfind("/")

    if last_slash == -1:
        # No slash found - file is in current directory
        return ""

    return filepath[:last_slash]


def _resolve_relative_to_current_file(relative_path):
    """Resolve a path relative to the current file's directory."""
    current_file = __file__

    # Get directory from __file__
    file_dir = _get_dirname(current_file)

    # If no directory (file in current directory), use cwd
    if not file_dir:
        file_dir = os.getcwd()

    # Handle absolute path
    if relative_path.startswith("/"):
        return relative_path

    return file_dir + "/" + relative_path


DEFAULT_LIB_PATH = _resolve_relative_to_current_file("build/libbclibc_ffi.so")
print(DEFAULT_LIB_PATH)

# ============================================================================
# Constants
# ============================================================================


class Status:
    OK = 0
    ERR_SOLVER_RUNTIME = 1
    ERR_OUT_OF_RANGE = 2
    ERR_ZERO_FINDING = 3
    ERR_INTERCEPTION = 4
    ERR_GENERIC = 5


class TrajFlag:
    NONE = 0
    ZERO_UP = 1
    ZERO_DOWN = 2
    ZERO = 3
    MACH = 4
    RANGE = 8
    APEX = 16
    ALL = 31
    MRT = 32


class TerminationReason:
    NO_TERMINATE = 0
    TARGET_RANGE_REACHED = 1
    MINIMUM_VELOCITY_REACHED = 2
    MAXIMUM_DROP_REACHED = 3
    MINIMUM_ALTITUDE_REACHED = 4
    HANDLER_REQUESTED_STOP = 5


class InterpKey:
    TIME = 0
    MACH = 1
    POS_X = 2
    POS_Y = 3
    POS_Z = 4
    VEL_X = 5
    VEL_Y = 6
    VEL_Z = 7


class IntegrationMethod:
    RK4 = 0
    EULER = 1


# ============================================================================
# Main library class
# ============================================================================


class BCLibc:
    def __init__(self, lib_path=DEFAULT_LIB_PATH):
        self._lib = ffi.open(lib_path)
        self._register_functions()

    def _register_functions(self):
        self.get_version = self._lib.func("s", "BCLIBCFFI_get_version", "")
        self.get_correction = self._lib.func("d", "BCLIBCFFI_get_correction", "dd")
        self.calculate_energy = self._lib.func("d", "BCLIBCFFI_calculate_energy", "dd")
        self.calculate_ogw = self._lib.func("d", "BCLIBCFFI_calculate_ogw", "dd")
        self.free_trajectory = self._lib.func("v", "BCLIBCFFI_free_trajectory", "p")

    def version(self):
        return self.get_version()

    def correction(self, distance_ft, offset_ft):
        return self.get_correction(distance_ft, offset_ft)

    def energy(self, bullet_weight_grain, velocity_fps):
        return self.calculate_energy(bullet_weight_grain, velocity_fps)

    def ogw(self, bullet_weight_grain, velocity_fps):
        return self.calculate_ogw(bullet_weight_grain, velocity_fps)


# ============================================================================
# Main function
# ============================================================================


def main():
    try:
        lib = BCLibc()
        print("BCLIBC Version:", lib.version())
        print("Correction at 100ft, 0ft offset:", lib.correction(100.0, 0.0), "rad")
        print("Energy (150gr, 2800fps):", lib.energy(150.0, 2800.0), "ft-lb")
        print("OGW (150gr, 2800fps):", lib.ogw(150.0, 2800.0), "lb")
        return 0
    except Exception as e:
        print("Error:", e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
