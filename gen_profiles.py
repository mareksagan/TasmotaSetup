#!/usr/bin/env python3
"""Generate profiles.bin and update _pf_names() in yc01.be from profiles.csv."""
import struct
import csv
import re
import sys

CSV_FILE = "profiles.csv"
BIN_FILE = "profiles.bin"
BE_FILE = "yc01.be"

# Record format: 1 byte name_len + 40 bytes name + 17 × 4-byte int32 (val*100)
RECORD_SIZE = 1 + 40 + 17 * 4  # 109 bytes

def parse_csv():
    profiles = []
    with open(CSV_FILE) as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            # Name is everything before the first numeric field
            name = None
            for i, field in enumerate(row):
                try:
                    float(field)
                    name = ', '.join(p.strip('" ').strip() for p in row[:i])
                    rest = row[i:]
                    break
                except ValueError:
                    pass
            if name is None:
                name = ', '.join(p.strip('" ').strip() for p in row)
                rest = []
            while len(rest) < 17:
                rest.append('')
            profiles.append((name, rest))
    return profiles

def generate_bin(profiles):
    with open(BIN_FILE, 'wb') as f:
        # Header: 2 bytes = number of profiles
        f.write(struct.pack('<H', len(profiles)))
        
        for name, vals in profiles:
            name_bytes = name.encode('utf-8')[:40]
            f.write(struct.pack('B', len(name_bytes)))
            f.write(name_bytes.ljust(40, b'\x00'))
            
            for v in vals[:17]:
                try:
                    iv = int(float(v) * 100) if v else 0
                except ValueError:
                    iv = 0
                f.write(struct.pack('<i', iv))

def update_berry(names):
    with open(BE_FILE, 'r') as f:
        content = f.read()
    
    # Generate compact _pf_names function (names packed per line)
    new_func = 'def _pf_names()\n    return ['
    line = ""
    for name in names:
        if line != "":
            line += ", "
        line += f'"{name}"'
        if len(line) > 90:
            new_func += line + ",\n        "
            line = ""
    if line != "":
        new_func += line
    new_func += ']\nend'
    
    # Replace existing function
    pattern = r'def _pf_names\(\)\s*return\s*\[.*?\]\s*end'
    new_content = re.sub(pattern, new_func, content, flags=re.DOTALL)
    
    with open(BE_FILE, 'w') as f:
        f.write(new_content)

if __name__ == '__main__':
    profiles = parse_csv()
    names = [p[0] for p in profiles]
    
    generate_bin(profiles)
    print(f"Generated {BIN_FILE}: {len(profiles)} profiles, {2 + len(profiles) * RECORD_SIZE} bytes")
    
    update_berry(names)
    print(f"Updated _pf_names() in {BE_FILE} with {len(names)} profiles")
    print("\nNow upload both files to Tasmota UFS:")
    print(f"  {BIN_FILE}")
    print(f"  {BE_FILE}")
