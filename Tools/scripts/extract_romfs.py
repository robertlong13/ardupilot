#!/usr/bin/env python3
'''
Extracts the ROMFS files from an ArduPilot ELF file using GDB.

(must have been built with symbol information, i.e. waf configure -g)
'''
import pexpect
import zlib
import os
import re


def load_romfs_with_gdb(elf_path):
    # Start GDB
    gdb = pexpect.spawn(f"arm-none-eabi-gdb {elf_path}")

    # Set timeout and logging
    gdb.timeout = 30
    gdb.logfile = None

    def gdb_command(cmd):
        gdb.sendline(cmd)
        gdb.expect_exact(cmd)  # Wait for the echo
        gdb.expect(r"\(gdb\) ")
        return gdb.before.decode("utf-8")

    # Optimize gdb's settings for pexpect
    gdb_command("set pagination off")
    gdb_command("set style enabled off")

    # Locate the AP_ROMFS::files symbol
    romfs_symbol_output = gdb_command("print AP_ROMFS::files").strip()
    if romfs_symbol_output.lower().startswith("no symbol"):
        gdb.terminate()
        raise ValueError(romfs_symbol_output + "\nMake sure you run waf configure with the -g flag")

    num_files_output = gdb_command("print sizeof(AP_ROMFS::files)/sizeof(*AP_ROMFS::files)")
    num_files_match = re.search(r"\$[0-9]+ = ([0-9]+)", num_files_output)
    if not num_files_match:
        gdb.terminate()
        raise ValueError(f"Failed to parse the number of files: {num_files_output}")
    num_files = int(num_files_match.group(1))

    romfs_entries = []

    # Read ROMFS entries until no more valid entries exist
    for index in range(num_files):
        # Fetch the current ROMFS entry
        entry_cmd = f"print AP_ROMFS::files[{index}]"
        entry_output = gdb_command(entry_cmd)

        # Grab the first quoted string as long as it comes before a comma
        filename_match = re.search(r"filename\s*=[^,]*\"([^,]+)\"", entry_output)
        if not filename_match:
            gdb.terminate()
            raise ValueError(f"Failed to parse filename: {entry_output}")

        size_match = re.search(r"compressed_size\s*=\s*([0-9]+)", entry_output)
        is_old_format = False
        if not size_match:
            # Try the old variable name
            size_match = re.search(r"size\s*=\s*([0-9]+)", entry_output)
            is_old_format = True
        if not size_match:
            gdb.terminate()
            raise ValueError(f"Failed to parse the compressed size: {entry_output}")

        contents_match = re.search(r"contents = 0x([0-9a-fA-F]+)", entry_output)
        if not contents_match:
            gdb.terminate()
            raise ValueError(f"Failed to parse the contents address: {entry_output}")

        crc_match = re.search(r"crc\s*=\s*(\d+)", entry_output)
        if not crc_match:
            gdb.terminate()
            raise ValueError(f"Failed to parse the CRC: {entry_output}")

        filename = filename_match.group(1)
        size = int(size_match.group(1))
        contents_ptr = int(contents_match.group(1), 16)
        crc = int(crc_match.group(1))

        # Read the compressed contents
        contents_output = gdb_command(f"x/{size}bx {contents_ptr}")
        compressed_data = bytes(int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})\s", contents_output))

        # Decompress the data
        if is_old_format:
            # Old format: Gzip with a header
            decompressor = zlib.decompressobj(wbits=16 + zlib.MAX_WBITS)  # Auto-detect Gzip header
        else:
            # New format: Raw DEFLATE stream
            decompressor = zlib.decompressobj(wbits=-15)  # Raw DEFLATE stream

        try:
            decompressed_data = decompressor.decompress(compressed_data) + decompressor.flush()
        except zlib.error as e:
            gdb.terminate()
            raise ValueError(f"Failed to decompress {filename}: {e}")

        # Verify the CRC
        if crc != crc32(decompressed_data):
            gdb.terminate()
            raise ValueError(f"CRC mismatch for {filename}")

        # Append the file data to the list
        romfs_entries.append({
            "filename": filename,
            "crc": crc,
            "content": decompressed_data
        })

    gdb.terminate()
    return romfs_entries


def write_romfs_files(romfs_entries, output_dir):
    for entry in romfs_entries:
        output_path = os.path.join(output_dir, entry["filename"])
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, 'wb') as output_file:
            output_file.write(entry["content"])
        print(f"Extracted and decompressed: {entry['filename']}")


def crc32(bytes, crc=0):
    '''crc32 equivalent to crc32_small() from AP_Math/crc.cpp'''
    for byte in bytes:
        crc ^= byte
        for i in range(8):
            mask = (-(crc & 1)) & 0xFFFFFFFF
            crc >>= 1
            crc ^= (0xEDB88320 & mask)
    return crc


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Extract and decompress ROMFS files using GDB.")
    parser.add_argument('elf_file', help="Path to the ELF file.")
    parser.add_argument('output_dir', help="Directory to save the extracted files.")

    args = parser.parse_args()

    romfs_entries = load_romfs_with_gdb(args.elf_file)
    write_romfs_files(romfs_entries, args.output_dir)
