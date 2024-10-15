#!/usr/bin/env python3
# pip install pillow to get the PIL module

import sys
from PIL import Image

def main(fn, id, file):
  orig_stdout = sys.stdout
  f = open(file, 'a')
  sys.stdout = f

  image = Image.open(fn)
  id_upper = str(id).upper()
  print("\n"
        "#define {id_upper}_WIDTH  {w}\n"
        "#define {id_upper}_HEIGHT {h}\n"
        "\n"
        "const uint8_t {id}_data[] = {{\n"
        .format(id_upper=id_upper,id=id, w=image.width, h=image.height), end='')
  for y in range(0, image.height):
    for x in range(0, (image.width + 7)//8 * 8):
      if x == 0:
        print("  ", end='')
      if x % 8 == 0:
        print("0b", end='')

      bit = '0'
      if x < image.width and image.getpixel((x,y)) != 0:
        bit = '1'
      print(bit, end='')

      if x % 8 == 7:
        print(",", end='')
    print()
  print("};")

if __name__ == '__main__':
    if len(sys.argv) < 4:
      print("Usage: {} <imagefile> <id>\n".format(sys.argv[0]), file=sys.stderr);
      sys.exit(1)
    fn = sys.argv[1]
    id = sys.argv[2]
    file = sys.argv[3]
    main(fn, id, file)
