# -*- coding: utf-8 -*-

import sys
import argparse
from math import *

from lib.export import Frame

id_func = lambda t: t

def parse_func(s):
  print "func: ", s
  return eval(s)
def parse_tuple(s):
  return tuple(int(v) for v in s.split(","))

parser = argparse.ArgumentParser(description='Using Math to generate drawings.')
parser.add_argument("--x", action = "store", dest = "x_func", default = id_func, type = parse_func, help = "x absciss coords function")
parser.add_argument("--y", action = "store", dest = "y_func", default = id_func, type = parse_func, help = "y absciss coords function")
parser.add_argument("--span", action = "store", dest = "span", default = 1000, type = int, help = "number of steps drawn")
parser.add_argument("--step", action = "store", dest = "step_value", default = 1.0, type = float, help = "parameter step size")
parser.add_argument("--scale", action = "store", dest = "scale", default = 1.0, type = float, help = "scaling factor")
parser.add_argument("--size", action = "store", dest = "size", default = 512, type = int, help = "scaling factor")
parser.add_argument("--offset", action = "store", dest = "offset", default = (256,256), type = parse_tuple, help = "scaling factor")
parser.add_argument("--output", action = "store", dest = "output", default = "mod.jpg", type = str, help = "output filename")


def generate_cartesian_curve(args, filename = "mod.jpg"):
  mod_frame = Frame(args.size, args.size)
  mod_frame.fill(1, 1, 1)

  mod_frame.draw_cartesian_curve(args.x_func, args.y_func, step_range = range(args.span), step_value = args.step_value, scale_factor = args.scale, offset = args.offset)

  mod_frame.export(filename)

if __name__ == "__main__":
  args = parser.parse_args(sys.argv[1:])
  generate_cartesian_curve(args, args.output)
