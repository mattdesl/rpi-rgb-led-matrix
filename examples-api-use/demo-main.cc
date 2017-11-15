// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
//
// This code is public domain
// (but note, once linked against the led-matrix library, this is
// covered by the GPL v2)
//
// This is a grab-bag of various demos and not very readable.
#include "led-matrix.h"
#include "threaded-canvas-manipulator.h"
#include "transformer.h"
#include "graphics.h"
#include <chrono>

#include <assert.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <chrono>

#include <algorithm>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "FastNoise.h"

using std::min;
using std::max;

#define TERM_ERR  "\033[1;31m"
#define TERM_NORM "\033[0m"

using namespace rgb_matrix;

int AMG88xx_I2CADD = 0x69;
int AMG88xx_PCTL = 0x00;
int AMG88xx_RST = 0x01;
int AMG88xx_FPSC = 0x02;
int AMG88xx_INTC = 0x03;
int AMG88xx_TTHL = 0x0E;
int AMG88xx_PIXEL_OFFSET = 0x80;
int AMG88xx_INITIAL_RESET = 0x3F;
int AMG88xx_PIXEL_ARRAY_SIZE = 64;
double AMG88xx_PIXEL_TEMP_CONVERSION = 0.25;
double AMG88xx_THERMISTOR_CONVERSION = 0.0625;

int i2c;
bool hasI2C = true;

volatile bool interrupt_received = false;
static void InterruptHandler(int signo) {
  interrupt_received = true;
}

float lerpf (float v0, float v1, float t) {
  return v0 * (1 - t) + v1 * t;
}

float smoothstep (float v0, float v1, float t) {
  float x = max(0.0f, min(1.0f, (t - v0) / (v1 - v0)));
  return x * x * (3.0 - 2.0 * x);
}

class ColorRGB {
public:
  int r;
  int g;
  int b;

  ColorRGB(int _r, int _g, int _b) {
    r = _r;
    g = _g;
    b = _b;
  }
  void copy (ColorRGB *other) {
    r = other->r;
    g = other->g;
    b = other->b;
  }
  void lerp (ColorRGB *color, float alpha) {
		r += (color->r - r) * alpha;
		g += (color->g - g) * alpha;
    b += (color->b - b) * alpha;
    r = max(0, min(255, r));
    g = max(0, min(255, g));
    b = max(0, min(255, b));
  }
};

class Sensor {
public:
  Sensor(int addr) {
    
  }
};

void writeI2C () {
  try {
    // enter normal mode
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_PCTL, (unsigned int)0) < 0) printf("error 1\n");
    // initial reset
    int initialReset = AMG88xx_INITIAL_RESET & ((1 << 8) - 1);
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_RST, (unsigned int)(initialReset)) < 0) printf("error 2\n");
    // disable interrupt mode
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_INTC, (unsigned int)0) < 0) printf("error 3\n");
    // set FPS to 10
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_FPSC, (unsigned int)0) < 0) printf("error 4\n");
  } catch (const std::exception& err) {
    printf("Could not open I2C connection...\n");
    hasI2C = false;
  }
}

float signedMag12ToFloat(uint16_t val) {
  //take first 11 bits as absolute val
  uint16_t absVal = (val & 0x7FF);
  return (val & 0x8000) ? 0 - (float)absVal : (float)absVal;
}

float readThermistor () {
  if (!hasI2C) return -1.0;
  else {
    // get current
    uint16_t rawVal = wiringPiI2CReadReg16(i2c, AMG88xx_TTHL);
    return signedMag12ToFloat(rawVal) * AMG88xx_THERMISTOR_CONVERSION;
  }
}

// Simple class that generates a rotating block on the screen.
class SDFGen : public ThreadedCanvasManipulator {
public:
  typedef std::chrono::high_resolution_clock clock_;
  typedef std::chrono::duration<double, std::ratio<1> > second_;
  std::chrono::time_point<clock_> lastTime;
  
  SDFGen(Canvas *m) : ThreadedCanvasManipulator(m) {}

  void Run() {
    int width = canvas()->width();
    int height = canvas()->height();
    float tick = 0;
    // ColorRGB *colorA = new ColorRGB(255, 255, 255);
    ColorRGB *colorRed = new ColorRGB(239, 21, 21);
    ColorRGB *colorBlue = new ColorRGB(18, 160, 226);
    ColorRGB *colorB = new ColorRGB(0, 0, 0);
    // ColorRGB *colorB = new ColorRGB(0, 0, 0);
    ColorRGB *colorTemp1 = new ColorRGB(0, 0, 0);
    ColorRGB *colorTemp2 = new ColorRGB(0, 0, 0);

    FastNoise noise; // Create a FastNoise object
    noise.SetNoiseType(FastNoise::Simplex); // Set the desired noise type

    writeI2C();
    printf("Running...\n");

    // start time
    lastTime = clock_::now();
    double currentTime = 0.0;
    double sensorTime = 0.0;
    double sensorFPS = 10.0;
    double sensorInterval = 1.0 / sensorFPS;

    // Color *colorB = new Color()
    while (running() && !interrupt_received) {
      std::chrono::time_point<clock_> newTime = clock_::now();
      double dt = std::chrono::duration_cast<second_>(newTime - lastTime).count();
      currentTime += dt;
      lastTime = newTime;
      sensorTime += dt;
      if (sensorTime > sensorInterval) {
        sensorTime = 0.0;

        printf("THERMISTOR %f\n", readThermistor());
      }

      usleep(15 * 1000);

      tick += 0.5;
      //If you want to do per-pixel modifications, it might look like this:
      for (int i=0; i<width * height; i++) {

        float x = floor(fmod((float)i, (float)width));
        float y = floor((float)i / width);
        // int y = i / width;
        // int x = i - width * y;
        // float radius = 0.5 + 0.5 * (noise.GetNoise(px * 100, py * 100, tick) * 0.5 + 0.5);
        float px = (x / (float)width) * 2.0 - 1.0;
        float py = -(y / (float)height) * 2.0 + 1.0;
        px *= width / (float)height;

        // float c = (noise.GetNoise(px * zoom, py * zoom, tick * 3.0) * 0.5 + 0.5);
        float dx = px;
        float dy = py;
        float dist = sqrt(dx * dx + dy * dy);
        
        // Number of sides of your shape
        int sides = 6;

        // Angle and radius from the current pixel
        float a = atan2(px, py) + M_PI + tick * 0.01;
        float r = (M_PI * 2.0) / (float)sides;
        
        // Shaping function that modulate the distance
        float zoom = lerpf(0.05, 4.0, (sin(tick * 0.05) * 0.5 + 0.5));
        float sdf = cosf(floor(0.5f + a / r) * r - a) * dist * (1.0f / 1.0) + tick * -0.02;
        float value = sinf(sdf * 18.0) * 0.5 + 0.5;
        value = smoothstep(0.0, 1.0, value);
        // float value = smoothstep(0.5, 0.51, sdf);
        // color = vec3(1.0-smoothstep(.4,.41,sdf));

        // float zoom = 3.0;//lerpf(10.0, 200.0, sinf(tick * 0.005) * 0.5 + 0.5);
        // float ripples = (noise.GetNoise(px * zoom, py * zoom, tick * 1.0) * 8.0);
        // float c = sinf(ripples * dist + tick * 0.05) * 0.5 + 0.5;

        //place pixel
        int C = (int)(value * 255);
        float blue = sinf(tick * 0.025) * 0.5 + 0.5;
        // colorTemp2->r = C;
        // colorTemp2->g = C;
        // colorTemp2->b = C;
        
        // colorTemp2->copy(colorRed);
        // colorTemp2->lerp(colorBlue, value);
        
        colorTemp1->copy(colorRed);
        colorTemp1->lerp(colorBlue, blue);
        colorTemp2->copy(colorTemp1);
        colorTemp2->lerp(colorB, value);
        canvas()->SetPixel(x, y, colorTemp2->r, colorTemp2->g, colorTemp2->b);
        // pixels[i] = (C << 16) | (C << 8) | C;
     }
    }
  }
};

// Simple class that generates a rotating block on the screen.
class SDFGen2 : public ThreadedCanvasManipulator {
public:
  SDFGen2(Canvas *m) : ThreadedCanvasManipulator(m) {}

  void Run() {
    int width = canvas()->width();
    int height = canvas()->height();
    float tick = 0;
    // ColorRGB *colorA = new ColorRGB(255, 255, 255);
    ColorRGB *colorRed = new ColorRGB(239, 21, 21);
    ColorRGB *colorBlue = new ColorRGB(18, 160, 226);
    ColorRGB *colorB = new ColorRGB(0, 0, 0);
    // ColorRGB *colorB = new ColorRGB(0, 0, 0);
    ColorRGB *colorTemp1 = new ColorRGB(0, 0, 0);
    ColorRGB *colorTemp2 = new ColorRGB(0, 0, 0);

    FastNoise noise; // Create a FastNoise object
    noise.SetNoiseType(FastNoise::Simplex); // Set the desired noise type
    
    // Color *colorB = new Color()
    while (running() && !interrupt_received) {
      usleep(15 * 1000);
      tick += 0.5;
      //If you want to do per-pixel modifications, it might look like this:
      for (int i=0; i<width * height; i++) {
        int y = i / width;
        int x = i - width*y;
        // float radius = 0.5 + 0.5 * (noise.GetNoise(px * 100, py * 100, tick) * 0.5 + 0.5);
        float px = (x / (float)width) * 2.0 - 1.0;
        float py = (y / (float)width) * 2.0 - 1.0;
        float zoom = lerpf(10.0, 200.0, sinf(tick * 0.005) * 0.5 + 0.5);
        float c = (noise.GetNoise(px * zoom, py * zoom, tick * 3.0) * 0.5 + 0.5);

        // float px = (2*(x/(float)width-.5f)) * (width/(float)height);
        // float py = (2*(y/(float)height-.5f));
        
        // float radius = 1;
        
        // // evaluate z-depth of sphere
        // float z = sqrt(radius * radius - px*px - py*py);
        // // float zOff = 2.5 * sinf(tick);
        // // z += zOff;

        // // evaluate normal of sphere
        // float dist = sqrt(px * px + py * py + z * z);
        // px/=dist; py/=dist; z/=dist;
        
        // //centered mouse position
        // // float mx = 0;
        // // float my = 0;
        // float mx = sinf(tick * 0.15) * 1.0;
        // float my = sinf(cosf(tick * 0.1)) * 1;
        
        // // float mx = mouseX/(float)width-.5f, my = mouseY/(float)height-.5f;
        
        // //normalize light direction
        // dist = sqrt(mx*mx + my*my + 1);
        // mx/=dist; my/=dist;

        // // N dot L and clamp
        // float c = max(0.0f, px * mx + py * my + z * (1 / dist));
        
        //place pixel
        // int C = (int)(c*255);
        float blue = sinf(tick * 0.025) * 0.5 + 0.5;
        colorTemp1->copy(colorRed);
        colorTemp1->lerp(colorBlue, blue);
        colorTemp2->copy(colorTemp1);
        colorTemp2->lerp(colorB, c);
        canvas()->SetPixel(x, y, colorTemp2->r, colorTemp2->g, colorTemp2->b);
        // pixels[i] = (C << 16) | (C << 8) | C;
     }
    }
  }
};

static int usage(const char *progname) {
  fprintf(stderr, "usage: %s <options> -D <demo-nr> [optional parameter]\n",
          progname);
  fprintf(stderr, "Options:\n");
  fprintf(stderr,
          "\t-D <demo-nr>              : Always needs to be set\n"
          "\t-L                        : Large display, in which each chain is 'folded down'\n"
          "\t                            in the middle in an U-arrangement to get more vertical space.\n"
          "\t-R <rotation>             : Sets the rotation of matrix. "
          "Allowed: 0, 90, 180, 270. Default: 0.\n"
          "\t-t <seconds>              : Run for these number of seconds, then exit.\n");


  rgb_matrix::PrintMatrixFlags(stderr);

  fprintf(stderr, "Demos, choosen with -D\n");
  fprintf(stderr, "\t0  - some rotating square\n");
  fprintf(stderr, "Example:\n\t%s -t 10 -D 1 runtext.ppm\n"
          "Scrolls the runtext for 10 seconds\n", progname);
  return 1;
}

int main(int argc, char *argv[]) {
  int runtime_seconds = -1;
  int demo = -1;
  int rotation = 0;
  bool large_display = false;

  printf("Opening i2C connection...\n");
  wiringPiSetup();
  try {
    if ((i2c = wiringPiI2CSetup(AMG88xx_I2CADD)) < 0) {
      printf("Error opening I2C channel...\n");
      hasI2C = false;
    }
  } catch (const std::exception& err) {
    printf("Could not open I2C connection...\n");
    hasI2C = false;
  }

  RGBMatrix::Options matrix_options;
  rgb_matrix::RuntimeOptions runtime_opt;

  // These are the defaults when no command-line flags are given.
  matrix_options.rows = 32;
  matrix_options.chain_length = 1;
  matrix_options.parallel = 1;

  // First things first: extract the command line flags that contain
  // relevant matrix options.
  if (!ParseOptionsFromFlags(&argc, &argv, &matrix_options, &runtime_opt)) {
    return usage(argv[0]);
  }

  int opt;
  while ((opt = getopt(argc, argv, "dD:t:r:P:c:p:b:m:LR:")) != -1) {
    switch (opt) {
    case 'D':
      demo = atoi(optarg);
      break;

    case 't':
      runtime_seconds = atoi(optarg);
      break;

    case 'R':
      rotation = atoi(optarg);
      break;

    case 'L':
      if (matrix_options.chain_length == 1) {
        // If this is still default, force the 64x64 arrangement.
        matrix_options.chain_length = 4;
      }
      large_display = true;
      break;

      // These used to be options we understood, but deprecated now. Accept
      // but don't mention in usage()
    case 'd':
      runtime_opt.daemon = 1;
      break;

    case 'r':
      matrix_options.rows = atoi(optarg);
      break;

    case 'P':
      matrix_options.parallel = atoi(optarg);
      break;

    case 'c':
      matrix_options.chain_length = atoi(optarg);
      break;

    case 'p':
      matrix_options.pwm_bits = atoi(optarg);
      break;

    case 'b':
      matrix_options.brightness = atoi(optarg);
      break;

    default: /* '?' */
      return usage(argv[0]);
    }
  }

  if (demo < 0) {
    fprintf(stderr, TERM_ERR "Expected required option -D <demo>\n" TERM_NORM);
    return usage(argv[0]);
  }

  if (rotation % 90 != 0) {
    fprintf(stderr, TERM_ERR "Rotation %d not allowed! "
            "Only 0, 90, 180 and 270 are possible.\n" TERM_NORM, rotation);
    return 1;
  }

  RGBMatrix *matrix = CreateMatrixFromOptions(matrix_options, runtime_opt);
  if (matrix == NULL)
    return 1;

  if (large_display) {
    // Mapping the coordinates of a 32x128 display mapped to a square of 64x64.
    // Or any other U-arrangement.
    matrix->ApplyStaticTransformer(UArrangementTransformer(
                                     matrix_options.parallel));
  }

  if (rotation > 0) {
    matrix->ApplyStaticTransformer(RotateTransformer(rotation));
  }

  printf("Size: %dx%d. Hardware gpio mapping: %s\n",
         matrix->width(), matrix->height(), matrix_options.hardware_mapping);

  Canvas *canvas = matrix;

  // The ThreadedCanvasManipulator objects are filling
  // the matrix continuously.
  ThreadedCanvasManipulator *image_gen = new SDFGen(canvas);
  // switch (demo) {
  // case 0:
  //   // image_gen = new SDFGen(canvas);
  //   // image_gen = new RotatingBlockGenerator(canvas);
  //   break;
  // }

  if (image_gen == NULL)
    return usage(argv[0]);

  // Set up an interrupt handler to be able to stop animations while they go
  // on. Note, each demo tests for while (running() && !interrupt_received) {},
  // so they exit as soon as they get a signal.
  signal(SIGTERM, InterruptHandler);
  signal(SIGINT, InterruptHandler);

  // Image generating demo is crated. Now start the thread.
  image_gen->Start();

  // Now, the image generation runs in the background. We can do arbitrary
  // things here in parallel. In this demo, we're essentially just
  // waiting for one of the conditions to exit.
  if (runtime_seconds > 0) {
    sleep(runtime_seconds);
  } else {
    // The
    printf("Press <CTRL-C> to exit and reset LEDs\n");
    while (!interrupt_received) {
      sleep(1); // Time doesn't really matter. The syscall will be interrupted.
    }
  }

  // Stop image generating thread. The delete triggers
  delete image_gen;
  delete canvas;

  printf("\%s. Exiting.\n",
         interrupt_received ? "Received CTRL-C" : "Timeout reached");
  return 0;
}
