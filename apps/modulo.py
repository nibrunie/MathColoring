# -*- coding: utf-8 -*-

import sys
import argparse
from math import cos, sin, tan, atan, atan2, pi

from lib.frame import PictureFrame


parser = argparse.ArgumentParser(description='Using Math to generate drawings.')
parser.add_argument("--modulo", action = "store", dest = "modulo", default = 80, type = int, help = "modulo")
parser.add_argument("--mult", action = "store", dest = "mult", default = 4, type = int, help = "mult")
parser.add_argument("--span", action = "store", dest = "span", default = 1000, type = int, help = "number of steps drawn")
parser.add_argument("--radius", action = "store", dest = "radius", default = 250, type = int, help = "radius value")
parser.add_argument("--size", action = "store", dest = "size", default = 512, type = int, help = "image size")
parser.add_argument("--output", action = "store", dest = "output", default = "mod.jpg", type = str, help = "output filename")


def generate_modulo(args, filename = "mod.jpg"):
  mod_frame = PictureFrame(args.size, args.size)
  mod_frame.fill(1, 1, 1)
  c_xy = args.size / 2
  mod_frame.draw_circle(c_xy, c_xy, args.radius, (0, 0, 0))

  c_x = c_xy
  c_y = c_xy
  radius = args.radius

  def get_mod_circle_point(i, mod):
    index = i % mod
    angle = index / float(mod) * 2.0 * pi
    x = c_x + cos(angle) * radius
    y = c_y + sin(angle) * radius
    return (x, y)
    
  for i in xrange(0, args.span):
    value = args.mult * i
    start_p = get_mod_circle_point(i, args.modulo)
    next_p = get_mod_circle_point(value, args.modulo)
    mod_frame.draw_segment(start_p, next_p)

  mod_frame.export(filename)

if __name__ == "__main__":
  args = parser.parse_args(sys.argv[1:])
  generate_modulo(args, args.output)
