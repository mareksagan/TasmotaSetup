#!/usr/bin/env python3
"""Generate profiles.bin from new_profiles.csv for yc01.be driver."""
import struct
import csv

CSV_FILE = "new_profiles.csv"
BIN_FILE = "profiles.bin"

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
    # Sort by name for binary search
    profiles.sort(key=lambda p: p[0])
    
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

if __name__ == '__main__':
    profiles = parse_csv()
    
    generate_bin(profiles)
    print(f"Generated {BIN_FILE}: {len(profiles)} profiles, {2 + len(profiles) * RECORD_SIZE} bytes")
    print(f"\nUpload both files to Tasmota UFS:")
    print(f"  {BIN_FILE}")
    print(f"  yc01.be")
