import numpy as np
import imageio
from PIL import Image, ImageDraw
from math import *


## Convert color from RGB float [0.0;1.0] format
#  to PIL's RGV int [0;255] formats
def get_pil_color(r = 0.0, g = 0.0, b = 0.0):
  return (int(r * 255), int(g * 255), int(b * 255))

class Frame:
  def __init__(self, w, h, mode = 'RGB'):
    self.w = w
    self.h = h
    self.buffer = Image.new('RGB', (w, h))
    self.draw_handle = ImageDraw.Draw(self.buffer)

  def export(self, filename):
    self.buffer.save(filename)

  def fill(self, r = 1.0, g = 1.0, b = 1.0):
    # ImageDraw.floodfill(self.buffer, (0,0), get_pil_color(r, g, b))
    self.draw_handle.rectangle([(0,0),(self.w,self.h)], get_pil_color(r, g, b))

  def draw_circle(self, x, y, radius, color):
    bounding_box = [(x - radius, y - radius), (x + radius, y + radius)]
    self.draw_handle.ellipse(bounding_box, outline = get_pil_color(*color))

  def draw_line(self, p0, p1, color = (0, 0, 0)):
    x0, y0 = p0
    x1, y1 = p1
    self.draw_handle.line([(x0,y0), (x1,y1)], fill = color, width = 1)

  def draw_polar_curve(self, start_point, radius_formulae, angle_range, steps = 1000, color = (0, 0, 0)):
    angle_start, angle_end = angle_range
    px, py = start_point
    points = []
    for i in xrange(steps):
      angle = angle_start + (i / float(steps)) * (angle_end - angle_start)
      radius = radius_formulae(angle)
      x = px + radius * cos(angle)
      y = py + radius * sin(angle)
      points.append((x, y))
    self.draw_handle.line(points, fill = get_pil_color(*color), width = 1)

  def draw_cartesian_curve(self, x_function, y_function, step_range = range(1000), step_value = 1, color = (0, 0, 0), scale_factor = 1.0, offset = (0, 0)):
    ox, oy = offset
    points = []
    for t in step_range[1:]:
      t_value = t * step_value
      x = scale_factor * x_function(t_value) + ox
      y = scale_factor * y_function(t_value) + oy
      points.append((x, y))
    self.draw_handle.line(points, fill = get_pil_color(*color), width = 1)



if __name__ == "__main__":
  my_frame = Frame(512, 512)
  my_frame.export("test.png")
  my_frame.fill(0, 1, 1)
  my_frame.export("test_fill.png")
  my_frame.draw_circle(256, 256, 32, (1, 0, 0))
  my_frame.export("test_circle.png")
  my_frame.draw_line((0, 256), (512, 256))
  my_frame.export("test_line.png")

  polar_frame = Frame(512, 512)
  polar_frame.fill(1, 1, 1)
  formulae0 = lambda theta: (200 * (cos(theta / 10.0) + 0.2))
  formulae1 = lambda theta: (200 * (cos(3.0 * theta / 10.0) + 0.0))
  polar_frame.draw_polar_curve((256, 256), formulae1, [-20 * pi, 20 * pi])
  polar_frame.export("polar.png")
  polar_frame.export("polar_curve.png")

    


