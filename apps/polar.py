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
parser.add_argument("--radius", action = "store", dest = "polar_func", default = id_func, type = parse_func, help = "radius polar equation")
parser.add_argument("--span", action = "store", dest = "span", default = 1000, type = int, help = "number of steps drawn")
parser.add_argument("--angle-range", action = "store", dest = "angle_range", default = [-pi,+pi], type = parse_tuple, help = "angle range")
parser.add_argument("--steps", action = "store", dest = "steps", default = 1000, type = int, help = "number of steps to draw")
parser.add_argument("--size", action = "store", dest = "size", default = 512, type = int, help = "scaling factor")
parser.add_argument("--offset", action = "store", dest = "offset", default = (256,256), type = parse_tuple, help = "scaling factor")
parser.add_argument("--output", action = "store", dest = "output", default = "mod.jpg", type = str, help = "output filename")


def generate_cartesian_curve(args, filename = "mod.jpg"):
  mod_frame = Frame(args.size, args.size)
  mod_frame.fill(1, 1, 1)

  mod_frame.draw_polar_curve(args.offset, args.polar_func, angle_range = range(args.span), steps = args.steps)

  mod_frame.export(filename)

if __name__ == "__main__":
  args = parser.parse_args(sys.argv[1:])
  generate_cartesian_curve(args, args.output)
