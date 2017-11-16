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

#include <deque>

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
#define AMG88xx_PIXEL_ARRAY_SIZE 64
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

float clamp (float v, float minVal, float maxVal) {
  return max(minVal, min(maxVal, v));
}

float lerp (float v0, float v1, float t) {
  return v0*(1-t)+v1*t;
}

float unlerp (float minVal, float maxVal, float value) {
  if (minVal == maxVal) {
    if (value > minVal) return 1;
    else return 0;
  }
  return (value - minVal) / (maxVal - minVal);
}

float unlerpClamp (float minVal, float maxVal, float value) {
  return unlerp(minVal, maxVal, clamp(value, minVal, maxVal));
}

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

void readPixels (float *buf) {
  // float converted;
	// uint8_t bytesToRead = min(size << 1, AMG88xx_PIXEL_ARRAY_SIZE << 1);
	// uint8_t rawArray[bytesToRead];
	// this->read(AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);
	
	for(int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++){
    uint16_t raw = wiringPiI2CReadReg16(i2c, AMG88xx_PIXEL_OFFSET + (i << 1));
    float converted = signedMag12ToFloat(raw) * AMG88xx_PIXEL_TEMP_CONVERSION;
    buf[i] = converted;
	}
}

class Vec2 {
public:
  float x;
  float y;

  Vec2(int _x, int _y) {
    x = _x;
    y = _y;
  }
  void copy (Vec2 *other) {
    x = other->x;
    y = other->y;
  }
  void set (float _x, float _y) {
    x = _x;
    y = _y;
  }
  void lerp (Vec2 *other, float alpha) {
		x += (other->x - x) * alpha;
		y += (other->y - y) * alpha;
  }
  float length () {
    return sqrt(x * x + y * y);
  }
  float lengthSq () {
    return x * x + y * y;
  }
};

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
  void setRGBFloat (float rf, float gf, float bf) {
    r = (int)(rf * 255);
    g = (int)(gf * 255);
    b = (int)(bf * 255);
  }
  void clampBytes () {
    r = clamp(r, 0, 255);
    g = clamp(g, 0, 255);
    b = clamp(b, 0, 255);
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
    clampBytes();
  }
};

// Simple class that generates a rotating block on the screen.
class SDFGen : public ThreadedCanvasManipulator {
public:
  typedef std::chrono::high_resolution_clock clock_;
  typedef std::chrono::duration<double, std::ratio<1> > second_;
  std::chrono::time_point<clock_> lastTime;
  Vec2 *tempVec = new Vec2(0, 0);

  ColorRGB *colorBlack = new ColorRGB(0, 0, 0);
  ColorRGB *colorBlue = new ColorRGB(0, 0, 255);
  ColorRGB *colorRed = new ColorRGB(255, 0, 0);
  
  // the raw 8x8 temperatures
  float temperatures[AMG88xx_PIXEL_ARRAY_SIZE];

  // tweened 8x8 temperatures
  float interpolatedTemperatures[AMG88xx_PIXEL_ARRAY_SIZE];

  // upscaled and smoothed temperatures
  float *upscaledTemperatures;

  float MINTEMP = 22; // 26
  float MAXTEMP = 30; // 32

  std::deque<float> movingAverages;
  float movingAverageTemp = MINTEMP;
  float targetTemp = 30;
  float currentAverage = MINTEMP;

  float newAverage = MINTEMP;
  float periodTime = 20;
  float interpolationSpeed = 0.15;

  double currentTime = 0.0;
  double sensorTime = 0.0;
  double sensorFPS = 10.0; // update thermal cam at 10 FPS
  double sensorInterval = 1.0 / sensorFPS;

  int averagePeriod = (int)(periodTime * sensorFPS);
  float newMovingAverageTemp = MINTEMP;
  FastNoise noise; // Create a FastNoise object

  SDFGen(Canvas *m) : ThreadedCanvasManipulator(m) {}

  void updatePixels () {
    readPixels(temperatures);

    // get the average temperature of the current set of pixels
    // and the min / max temperature
    newAverage = 0.0;
    // float currentMin = MAXTEMP;
    // float currentMax = MINTEMP;
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      float temperature = temperatures[i];
      newAverage += temperature;
      // if (temperature < currentMin) currentMin = temperature;
      // if (temperature > currentMax) currentMax = temperature;
    }
    newAverage /= AMG88xx_PIXEL_ARRAY_SIZE;

    // compute the new rolling averages
    movingAverages.push_back(newAverage);
    if (movingAverages.size() > averagePeriod) {
      movingAverages.pop_front();
    }

    int count = movingAverages.size();
    newMovingAverageTemp = 0;
    for (int i = 0; i < count; i++) {
      newMovingAverageTemp += movingAverages[i];
    }
    newMovingAverageTemp /= count;
  }

  void resizePixels () {

  }

  void interpolateTemperature () {
    movingAverageTemp = lerp(movingAverageTemp, newMovingAverageTemp, interpolationSpeed);
    currentAverage = lerp(currentAverage, newAverage, interpolationSpeed);
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      interpolatedTemperatures[i] = lerp(interpolatedTemperatures[i], temperatures[i], interpolationSpeed);
    }
  }

  void Run () {
    int width = canvas()->width();
    int height = canvas()->height();
    ColorRGB *fragColor = new ColorRGB(0, 0, 0);

    noise.SetNoiseType(FastNoise::Simplex); // Set the desired noise type

    // start time
    lastTime = clock_::now();

    // clear temperatures
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      temperatures[i] = MINTEMP;
      interpolatedTemperatures[i] = MINTEMP;
    }

    while (running() && !interrupt_received) {
      std::chrono::time_point<clock_> newTime = clock_::now();
      double dt = std::chrono::duration_cast<second_>(newTime - lastTime).count();
      currentTime += dt;
      lastTime = newTime;
      sensorTime += dt;
      if (hasI2C && sensorTime > sensorInterval) {
        sensorTime = 0.0;
        updatePixels();
      }

      usleep(15 * 1000);

      // interpolate toward the new rolling averages
      interpolateTemperature();
      // printf("temp %f\n", newAverage);

      for (int i = 0; i < width * height; i++) {
        int x = (int)(fmod((float)i, (float)width));
        int y = (int)((float)i / width);

        // flip horizontally
        // x = width - x - 1;

        // int y = i / width;
        // int x = i - width * y;
        // float radius = 0.5 + 0.5 * (noise.GetNoise(px * 100, py * 100, tick) * 0.5 + 0.5);
        float px = (x / (float)width);
        float py = (y / (float)height);
        fragColor->copy(colorBlack);
        pixelShader(fragColor, x, y, px, py, (float)width, (float)height, i);
        fragColor->clampBytes();

        // flip image before rendering
        int dstX = x;
        int dstY = height - y - 1;
        canvas()->SetPixel(dstX, dstY, fragColor->r, fragColor->g, fragColor->b);
      }
      // float cols = 8;
      // float rows = 8;
      // for (int i = 0; i < cols * rows; i++) {
      //   int x = (int)(fmod((float)i, (float)cols));
      //   int y = (int)((float)i / cols);
      //   float temperature = interpolatedTemperatures[i];
      //   float v = unlerpClamp(movingAverageTemp, targetTemp, temperature);
      //   fragColor->setRGBFloat(v, v, v);
      //   canvas()->SetPixel(x, y, fragColor->r, fragColor->g, fragColor->b);
      // }
    }
  }

  void pixelShader (ColorRGB *fragColor, int x, int y, float xCoord, float yCoord, float width, float height, int index) {
    float aspect = width / height;

    int srcWidth = 8;
    int srcHeight = 8;
    int xRatio = (int)((srcWidth << 16) / width) + 1;
    int yRatio = (int)((srcHeight << 16) / height) + 1;

    int srcX = ((x * xRatio) >> 16);
    int srcY = ((y * yRatio) >> 16);
    float temp = interpolatedTemperatures[(srcY * srcWidth) + srcX];
    float interaction = unlerpClamp(movingAverageTemp, targetTemp, temp);

    float zoom = 100;
    float c = (noise.GetNoise(xCoord * zoom * aspect, yCoord * zoom, currentTime * 50) * 0.5 + 0.5);
    fragColor->copy(colorBlue);
    fragColor->lerp(colorBlack, c);

    fragColor->lerp(colorRed, interaction * lerp(0.85, 1.0, c));
    // fragColor->setRGBFloat(distance, distance, distance);

    // float distance = unlerpClamp(movingAverageTemp, targetTemp, currentAverage);
    
    // float distance = 0.0;
    // float cols = 8;
    // float rows = 8;
    // for (int i = 0; i < cols * rows; i++) {
    //   float srcX = floor(fmod((float)i, (float)cols));
    //   float srcY = floor((float)i / cols);
    //   float tempScale = unlerpClamp(movingAverageTemp, targetTemp, interpolatedTemperatures[i]);

    //   float spacing = 0.35;
    //   float dstX = spacing * ((srcX / (cols-1)) * 2 - 1);
    //   float dstY = spacing * ((srcY / (rows-1)) * 2 - 1);
    //   float radius = 0.2 * tempScale;
    //   float smoothness = 0.2;
    //   tempVec->set((xCoord - 0.5 + dstX) * aspect, (yCoord - 0.5 + dstY));

    //   float dist = tempVec->length();
    //   float circleDist = smoothstep(radius, radius - smoothness, dist);
    //   distance += circleDist;
    // }

    // tempVec->set((xCoord - 0.5 + 0) * aspect, (yCoord - 0.5 + 0));
    // float dist = tempVec->length();
    // distance += smoothstep(0.5, 0.45, dist);

    // px *= width / (float)height;

    // // float c = (noise.GetNoise(px * zoom, py * zoom, tick * 3.0) * 0.5 + 0.5);
    // float dx = px;
    // float dy = py;
    // float dist = sqrt(dx * dx + dy * dy);
    
    // // Number of sides of your shape
    // int sides = 6;

    // // Angle and radius from the current pixel
    // float a = atan2(px, py) + M_PI + tick * 0.01;
    // float r = (M_PI * 2.0) / (float)sides;
    
    // // Shaping function that modulate the distance
    // float zoom = lerpf(0.05, 4.0, (sin(tick * 0.05) * 0.5 + 0.5));
    // float sdf = cosf(floor(0.5f + a / r) * r - a) * dist * (1.0f / 1.0) + tick * -0.02;
    // float value = sinf(sdf * 18.0) * 0.5 + 0.5;
    // value = smoothstep(0.0, 1.0, value);
    // // float value = smoothstep(0.5, 0.51, sdf);
    // // color = vec3(1.0-smoothstep(.4,.41,sdf));

    // // float zoom = 3.0;//lerpf(10.0, 200.0, sinf(tick * 0.005) * 0.5 + 0.5);
    // // float ripples = (noise.GetNoise(px * zoom, py * zoom, tick * 1.0) * 8.0);
    // // float c = sinf(ripples * dist + tick * 0.05) * 0.5 + 0.5;

    // //place pixel
    // int C = (int)(value * 255);
    // float blue = sinf(tick * 0.025) * 0.5 + 0.5;
    // // colorTemp2->r = C;
    // // colorTemp2->g = C;
    // // colorTemp2->b = C;
    
    // // colorTemp2->copy(colorRed);
    // // colorTemp2->lerp(colorBlue, value);
    
    // colorTemp1->copy(colorRed);
    // colorTemp1->lerp(colorBlue, blue);
    // colorTemp2->copy(colorTemp1);
    // colorTemp2->lerp(colorB, value);
  }

  void Run2() {
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

    printf("Running...\n");

    // start time
    lastTime = clock_::now();
    double currentTime = 0.0;
    double sensorTime = 0.0;
    double sensorFPS = 10.0;
    double sensorInterval = 1.0 / sensorFPS;
    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

    // Color *colorB = new Color()
    while (running() && !interrupt_received) {
      std::chrono::time_point<clock_> newTime = clock_::now();
      double dt = std::chrono::duration_cast<second_>(newTime - lastTime).count();
      currentTime += dt;
      lastTime = newTime;
      sensorTime += dt;
      if (sensorTime > sensorInterval) {
        sensorTime = 0.0;
        readPixels(pixels);
        printf("PIXELS %f %f\n", pixels[0], pixels[1]);
        // printf("THERMISTOR %f\n", readThermistor());
      }

      usleep(15 * 1000);

      tick += 0.5;
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
  int demo = 0;
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
  writeI2C();

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
