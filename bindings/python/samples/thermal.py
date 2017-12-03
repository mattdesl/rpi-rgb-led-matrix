#!/usr/bin/env python
import time
import math
from samplebase import SampleBase
from rgbmatrix import graphics
import numpy as np
from scipy.interpolate import griddata
from Adafruit_AMG88xx import Adafruit_AMG88xx
from colour import Color

#low range of the sensor (this will be blue on the screen)
MINTEMP = 21 # 26

#high range of the sensor (this will be red on the screen)
MAXTEMP = 26 # 32

#how many color values we can have
COLORDEPTH = 1024

points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
grid_x, grid_y = np.mgrid[0:7:32j, 0:7:32j]

#sensor is an 8x8 grid so lets do a square

#the list of colors we can choose from
blue = Color("indigo")
colors = list(blue.range_to(Color("red"), COLORDEPTH))

#create the array of colors
colors = [(int(c.red * 255), int(c.green * 255), int(c.blue * 255)) for c in colors]

MAX_MARCHING_STEPS = 100
MIN_DIST = 0.0
MAX_DIST = 6.0
EPSILON = 0.0001

#some utility functions
def clamp(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def smoothstep (min_val, max_val, value):
    x = max(0, min(1, (value-min_val)/(max_val-min_val)))
    return x * x * (3 - 2 * x)

def lerp (v0, v1, t):
    return v0*(1-t)+v1*t

def unlerp (min_val, max_val, value):
    return (value - min_val) / (max_val - min_val)

def unlerpClamp (min_val, max_val, value):
    return unlerp(min_val, max_val, clamp(value, min_val, max_val))

def v_length (vec):
    return math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])

def v_set (vec, x, y, z):
    vec[0] = x
    vec[1] = y
    vec[2] = z
    return vec

def v_normalize (vec):
    lenSq = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]
    if lenSq != 0:
        mag = math.sqrt(lenSq)
        vec[0] /= mag
        vec[1] /= mag
        vec[2] /= mag
    return vec

def v_ray_direction (array, fieldOfViewRad, x, y, width, height):
    dx = round(x - width / 2) + 0.5
    dy = round(y - height / 2) + 0.5
    z = height / math.tan(fieldOfViewRad / 2)
    array[0] = dx
    array[1] = dy
    array[2] = -z
    v_normalize(array)
    return array

def v_lerp (out, a, b, t):
    out[0] = lerp(a[0], b[0], t)
    out[1] = lerp(a[1], b[1], t)
    out[2] = lerp(a[2], b[2], t)
    return out

def sphere (p):
    return v_length(p) - 1

def march (vector, eye, direction, depth):
    vector[0] = eye[0] + direction[0] * depth
    vector[1] = eye[1] + direction[1] * depth
    vector[2] = eye[2] + direction[2] * depth
    return vector

def clampColor (color):
    color[0] = clamp(int(color[0]), 0, 255)
    color[1] = clamp(int(color[1]), 0, 255)
    color[2] = clamp(int(color[2]), 0, 255)
    return color

class Thermal(SampleBase):
    def __init__(self, *args, **kwargs):
        super(Thermal, self).__init__(*args, **kwargs)
        self.sensor = None
        self.updatePixels()
        self.reloadSensor()

    def scene (self, p):
        p[2] += math.sin(self.iTime) * 0.5 + 0.5
        return sphere(p)

    def sdf (self, eye, direction, start, end, tempVector):
        depth = start
        for i in range(0, MAX_MARCHING_STEPS):
            march(tempVector, eye, direction, depth)
            dist = self.scene(tempVector)
            if dist < EPSILON:
                return depth
            depth += dist
            if depth >= end:
                return end
        return end

    # def run_sdf (self):
    #     v_ray_direction(direction, fov, x, y, width, height)
    #     dist = self.sdf(eye, direction, MIN_DIST, MAX_DIST, tempVector)
    #     if dist > MAX_DIST - EPSILON:
    #         # no hit, black
    #         v_set(color, 0, 0, 0)
    #     else:
    #         #hit
    #         v_set(color, 255, 0, 0)
    #     march(position, eye, direction, dist)

    def reloadSensor (self):
        if self.sensor is None:
            try:
                self.sensor = Adafruit_AMG88xx()
            except:
                print('Did not start sensor, will try again later.')

    def updatePixels (self):
        if self.sensor is not None:
            try:
                self.sensorPixels = self.sensor.readPixels()
                self.sensorPixels = [unlerpClamp(MINTEMP, MAXTEMP, p) for p in self.sensorPixels]
                print('new thermistor %f' % self.sensor.readThermistor())
            except:
                print('Could not read pixels.')
        # print(self.sensorPixels)
        # self.sensorAverage = sum(self.sensorPixels) / float(len(self.sensorPixels))

    def run(self):
        offset_canvas = self.matrix.CreateFrameCanvas()
        fov = 45 * math.pi / 180
        width = self.matrix.width
        height = self.matrix.height
        direction = [ 0, 0, 0 ]
        eye = [ 0, 0, 5 ]
        colorArray = [ [0, 0, 0] for x in range(0, width * height) ]
        tempVector = [ 0, 0, 0 ]
        position = [ 0, 0, 0 ]
        totalPixels = width * height
        curTime = 0
        lastTime = time.time()
        sensorFPS = 10
        sensorTime = 0
        sensorInterval = 1 / sensorFPS

        red = [ 0, 0, 100 ]
        blue = [ 0, 0, 0 ]
        # blue = [ 57, 215, 255 ]
        # red = [ 25, 26, 250 ]
        color = [ 0, 0, 0 ]

        while True:
            now = time.time()
            dt = now - lastTime
            lastTime = now
            curTime += dt
            sensorTime += dt
            if sensorTime > sensorInterval:
                sensorTime = 0
                self.updatePixels()
            
            # pixelWidth = 8
            # for i in range(0, pixelWidth * pixelWidth):
            #     x = int(i % pixelWidth)
            #     y = int(i / pixelWidth)

            #     sensorIndex = i = x + (y * pixelWidth)
            #     pixel = self.sensorPixels[sensorIndex]
            #     v_lerp(color, blue, red, pixel)
            #     clampColor(color)
            #     (r, g, b) = color
            #     offset_canvas.SetPixel(x, y, r, g, b)

            for i in range(0, totalPixels):
                x = int(i % width)
                y = int(i / width)
                
                # (r, g, b) = self.sensorColor
                # r = 

                circles = 15
                totalDist = 0
                for c in range(0, circles):
                    offset = float(c) / float(max(1, circles - 1))
                    xoffset = math.sin(curTime * 0.1 + c) * 0.5
                    dx = (x / float(width)) - 0.5 - xoffset
                    dy = (y / float(height)) - 0.5
                    dx *= width / height
                    radius = 0.15
                    dist = math.sqrt(dx * dx + dy * dy)
                    dist = 0.0 if dist > radius else 1.0
                    totalDist += dist
                totalDist = clamp(totalDist, 0, 1)
                v_lerp(color, blue, red, totalDist)
                clampColor(color)
                (r, g, b) = color
                # r = math.floor(255 * (math.sin(curTime * 1) * 0.5 + 0.5))
                # g = 0
                # b = 0
                offset_canvas.SetPixel(x, y, r, g, b)
            
            # time.sleep(0.05)
            offset_canvas = self.matrix.SwapOnVSync(offset_canvas)


        # while True:
        #     tick += 0.1
        #     # self.matrix.width
        #     width = 8
        #     height = 8

            # pixels = sensor.readPixels()
            # pixels = [map(p, MINTEMP, MAXTEMP, 0, COLORDEPTH - 1) for p in pixels]
            

        #     bicubic = griddata(points, pixels, (grid_x, grid_y), method='cubic')

        #     #draw everything
        #     for ix, row in enumerate(bicubic):
        #         for jx, pixel in enumerate(row):
        #             pygame.draw.rect(lcd, colors[constrain(int(pixel), 0, COLORDEPTH- 1)], (displayPixelHeight * ix, displayPixelWidth * jx, displayPixelHeight, displayPixelWidth))
        
        #     # for x in range(0, width):
        #     #     for y in range(0, height):
        #     #         i = x + (y * width)
        #     #         pixel = pixels[i]
        #     #         rgb = colors[constrain(int(pixel), 0, COLORDEPTH- 1)]
        #     #         offset_canvas.SetPixel(x, y, rgb[0], rgb[1], rgb[2])
        #     time.sleep(0.05)
        #     offset_canvas = self.matrix.SwapOnVSync(offset_canvas)


# Main function
if __name__ == "__main__":
  prog = Thermal()
  if (not prog.process()):
    prog.print_help()

