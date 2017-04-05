import numpy as np
import imageio
from PIL import Image, ImageDraw
from math import *
from stl import mesh
import numpy


## Convert color from RGB float [0.0;1.0] format
#  to PIL's RGV int [0;255] formats
def get_pil_color(r = 0.0, g = 0.0, b = 0.0):
  return (int(r * 255), int(g * 255), int(b * 255))

## Generic Frame class 
class Frame(object):
  ## draw a line of connected segments
  # @param point_list is a list of points, pair of coordinates
  # @param kw dict of extra parameters (implementation dependent)
  def draw_lines(self, point_list, **kw):
    max_norm = 0.0
    min_norm = 100.0
    sum_norm = 0.0
    for p0, p1 in zip(point_list[:-1], point_list[1:]):
      A = Point(*p0)
      B = Point(*p1)
      V = B - A
      norm = V.get_norm()
      max_norm = max(norm, max_norm)
      min_norm = min(norm, min_norm)
      sum_norm += norm
      self.draw_segment(p0, p1, **kw)
    print "max norm is {}\n avg norm is {}".format(max_norm, sum_norm / (len(point_list) - 1))
    print "min norm is {}".format(min_norm)

  ## Elementary segment generation
  def draw_segment(self, p0, p1, **kw):
    ## This method must be overriden by sub-class
    raise NotImplemented

  def get_polar_point(self, start_point, radius_formulae, angle):
    px, py = start_point
    radius = radius_formulae(angle)
    x = px + radius * cos(angle)
    y = py + radius * sin(angle)
    return x, y
    

  ## Generate a polar curve drawing
  #  @param max_seg is the maximal lenth allowed for a segment
  #         if one exceeds these threshold the function will try
  #         to divide it
  def draw_polar_curve(self, start_point, radius_formulae, angle_range, steps = 1000, max_seg = 1.0, **kw):#color = (0, 0, 0)):
    angle_start, angle_end = angle_range
    px, py = start_point
    points = []
    first_point = self.get_polar_point(start_point, radius_formulae, angle_start)
    points.append(first_point)
    angle_increment = 1.0 / float(steps) * (angle_end - angle_start)
    angle = angle_start
    for i in xrange(1, steps):
      local_increment = angle_increment
      #angle = angle_start + (i / float(steps)) * (angle_end - angle_start)
      x, y = self.get_polar_point(start_point, radius_formulae, angle + local_increment)
      # previous point 
      px, py = points[-1]
      while (Point(px, py) - Point(x, y)).get_norm() > max_seg:
        local_increment /= 2.0
        x, y = self.get_polar_point(start_point, radius_formulae, angle + local_increment)
      angle += local_increment
      points.append((x, y))
    self.draw_lines(points, **kw)
    #self.draw_handle.line(points, fill = get_pil_color(*color), width = 1)

  ## Generate a cartesian curve drawing
  def draw_cartesian_curve(self, x_function, y_function, step_range = range(1000), step_value = 1, scale_factor = 1.0, offset = (0, 0), **kw):
    ox, oy = offset
    points = []
    for t in step_range[1:]:
      t_value = t * step_value
      x = scale_factor * x_function(t_value) + ox
      y = scale_factor * y_function(t_value) + oy
      points.append((x, y))
    self.draw_lines(points, **kw)
    #self.draw_handle.line(points, fill = get_pil_color(*color), width = 1)

  ## Save picture in an output file
  def export(self, filename):
    raise NotImplemented

class Vector(object):
  def __init__(self, vx, vy, vz):
    self.vx = vx
    self.vy = vy
    self.vz = vz

  def vector_prod(self, v2):
    px = self.vy * v2.vz - self.vz * v2.vy
    py = self.vz * v2.vx - self.vx * v2.vz
    pz = self.vx * v2.vy - self.vy * v2.vx
    return Vector(px, py, pz)

  def dot_prod(self, v2):
    px = self.vx * v2.vx
    py = self.vy * v2.vy
    pz = self.vz * v2.vz 
    return px + py + pz

  def get_norm(self):
    return sqrt(self.vx**2 + self.vy**2 + self.vz**2)

  def normalize(self):
    norm = self.get_norm()
    new_vector = Vector(
      self.vx / norm,
      self.vy / norm,
      self.vz / norm 
    )
    return new_vector

  ## return vector orthogonal to @p self
  #  works only in plane z=0
  def get_orth(self):
    assert self.vz == 0.0
    return Vector(-self.vy, self.vx, self.vz)

  def scale(self, factor):
    return Vector(factor * self.vx, factor * self.vy, factor * self.vz)

  def __add__(self, v2):
    return Vector(
      self.vx + v2.vx,
      self.vy + v2.vy,
      self.vz + v2.vz
    )

  def __sub__(self, v2):
    return Vector(
      self.vx - v2.vx,
      self.vy - v2.vy,
      self.vz - v2.vz
    )


class Point(object):
  def __init__(self, x, y, z = 0.0):
    self.x = x
    self.y = y 
    self.z = z

  def get_coords(self):
    return self.x, self.y, self.z

  ## self - p2
  def __sub__(self, p2):
    if isinstance(p2, Point):
      return Vector(self.x - p2.x, self.y - p2.y, self.z - p2.z)
    elif isinstance(p2, Vector):
      return Point(self.x - p2.vx, self.y - p2.vy, self.z - p2.vz)
    else:
      raise NotImplemented

  def __add__(self, p2):
    if isinstance(p2, Vector):
      return Point(self.x + p2.vx, self.y + p2.vy, self.z + p2.vz)
    else:
      raise NotImplemented

## Frame class to generate 3D Objects (such as STL description)
class Mesh3DFrame(Frame):
  def __init__(self):
    self.point_map = {} 
    self.faces = []

  ## Return the normal to the vector @p V at the point @p A
  # @param V is the tangeant vector to the trajectory
  def get_point_normals(self, A, V, z = 0.0, width = 1.0):
    if False and A.get_coords() in self.point_map:
      return self.point_map[A.get_coords()]
    else:
      N = V.get_orth().normalize()
      E = A + N.scale(width)
      H = A - N.scale(width)
      #E = [ax + nx, ay + ny, z]
      # F = [bx + nx, by + ny, z]
      # G = [bx - nx, by - ny, z]
      # H = [ax - nx, ay - ny, z]
      self.point_map[A.get_coords()] = E, H
      return E, H
  
  ## draw a rectangular face, made of two triangles
  #  around the segment [p0, p1]
  def draw_segment(self, p0, p1, width = 5.0, z = 0.0):
    assert p0 != p1
    # compute n, vector orthogonal to p0, p1
    ax,ay = p0
    bx,by = p1
    A = Point(ax, ay, z)
    B = Point(bx, by, z)
    E, H = self.get_point_normals(A, B - A, z, width = width)
    F, G = self.get_point_normals(B, (B - A), z, width = width) 
    subface_list = [
      [H, G, E],
      [E, G, F]
    ]
    for face in subface_list:
      np_face = numpy.array([P.get_coords() for P in face])
      self.faces.append(np_face)

  def draw_lines(self, point_list, **kw):
    max_norm = 0.0
    min_norm = 100.0
    sum_norm = 0.0

    z = 0.0
    width = kw["width"]

    for p0, p1, p2 in zip(point_list[:-2], point_list[1:], point_list[2:]):
      A = Point(*p0)
      B = Point(*p1)
      C = Point(*p2)
      V = B - A
      norm = V.get_norm()
      max_norm = max(norm, max_norm)
      min_norm = min(norm, min_norm)
      sum_norm += norm
      #
      E, H = self.get_point_normals(A, V, z, width = width)
      F, G = self.get_point_normals(B, V, z, width = width) 
      self.draw_segment(p0, p1, **kw)
    print "max norm is {}\n avg norm is {}".format(max_norm, sum_norm / (len(point_list) - 1))
    print "min norm is {}".format(min_norm)


  def export(self, filename):
    # building mesh
    num_faces = len(self.faces)
    curve_mesh = mesh.Mesh(numpy.zeros(num_faces, dtype = mesh.Mesh.dtype))
    for i, f in enumerate(self.faces):
      for j in xrange(3):
        curve_mesh.vectors[i][j] = numpy.array(f[j])
    # exporting mesh to file
    curve_mesh.save(filename)


## 2D-Picture Frame class
class PictureFrame(Frame):
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

  ## Update generic argument dictionnary
  #  with PictureFrame specific argument
  def update_arg_dict(self, **kw):
    if not "color" in kw:
      kw.update({"color": (0, 0, 0)})
    return kw

  def draw_segment(self, p0, p1, **kw):
    self.draw_line(p0, p1, **self.update_arg_dict(**kw))

  def draw_line(self, p0, p1, color = (0, 0, 0)):
    x0, y0 = p0
    x1, y1 = p1
    self.draw_handle.line([(x0,y0), (x1,y1)], fill = color, width = 1)

  def draw_lines(self, point_list, **kw):
    if not "color" in kw:
      kw.update({"color": (0, 0, 0)})
    self.draw_handle.line(point_list, fill = get_pil_color(*kw["color"]), width = 1)





if __name__ == "__main__":
  my_frame = PictureFrame(512, 512)
  my_frame.export("test.png")
  my_frame.fill(0, 1, 1)
  my_frame.export("test_fill.png")
  my_frame.draw_circle(256, 256, 32, (1, 0, 0))
  my_frame.export("test_circle.png")
  my_frame.draw_line((0, 256), (512, 256))
  my_frame.export("test_line.png")

  polar_frame = PictureFrame(512, 512)
  polar_frame.fill(1, 1, 1)
  formulae0 = lambda theta: (200 * (cos(theta / 10.0) + 0.2))
  formulae1 = lambda theta: (200 * (cos(3.0 * theta / 10.0) + 0.0))
  polar_frame.draw_polar_curve((256, 256), formulae1, [-20 * pi, 20 * pi])
  polar_frame.export("polar.png")
  polar_frame.export("polar_curve.png")

  # 3D frame
  polar_3d = Mesh3DFrame()
  polar_3d.draw_polar_curve((0, 0), formulae1, [-20 * pi, 20 * pi], steps = 20000, width = 1.0)
  polar_3d.export("polar.stl")

    


