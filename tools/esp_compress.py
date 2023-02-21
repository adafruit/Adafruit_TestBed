import sys
import zlib
import hashlib
from pathlib import Path
import os.path

output_header = 'esp_binaries.h'

# Argument is path to folder containing .bin files
# This script will create .bin.gz file with the same name to the folder
# and also print out C array of the gz along with MD5 checksum
dir = ''
if len(sys.argv) == 1:
    print("No [DIRS] arguments, use current directory as default")
    dir = Path('./')
else:
    # only take 1 argument
    dir = Path(sys.argv[2])

def print_carray(f, payload):
    while len(payload) > 0:
        f.write('\n    ');
        f.write(', '.join('0x{:02x}'.format(x) for x in payload[0:8]))
        f.write(',')
        payload = payload[8:]
    f.write('\n')

with open(output_header, 'w') as fc:
    # print typedef struct
    fc.write('// Generated by tools/esp_compress.py\n')
    file_list = list(dir.glob('*.bin'))
    fc.write('#define ESP_BINARIES_COUNT ({})\n\n'.format(len(file_list)))
    for fname in file_list:
        with open(fname, 'rb') as fi:
            image = fi.read()
            zimage = zlib.compress(image, 9)
            fzname = fname.with_suffix('.bin.gz')
            md5 = hashlib.md5(image)

            # write .gz file
            with open(fzname, 'wb') as fz:
                fz.write(zimage)

            # write to c header file
            var = os.path.basename(fname.with_suffix(''))

            # bin gz contents
            fc.write('const uint8_t _{}_gz[{}] = {{'.format(var, len(zimage)))
            print_carray(fc, zimage)
            fc.write('};\n\n')

            fc.write('const esp32_zipfile_t {} = {{\n'.format(var))
            fc.write('  .name = "{}",\n'.format(var))
            fc.write('  .data = _{}_gz,\n'.format(var))
            fc.write('  .compressed_len = {},\n'.format(len(zimage)))
            fc.write('  .uncompressed_len = {},\n'.format(len(image)))
            fc.write('  .md5 = {')
            print_carray(fc, md5.digest())
            fc.write('  },\n')
            fc.write('};\n')
            fc.write('\n')

            print("Compressed {}: {} to {} bytes, deflation: {:.2f}%".format(fname, len(image), len(zimage), 100*(1-len(zimage)/len(image))))
