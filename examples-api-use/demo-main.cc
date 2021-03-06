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

float MINTEMP = 20; // 26
float MAXTEMP = 34; // 32
float CLEAN_RANGE_MIN = -60;
float CLEAN_RANGE_MAX = 250;
float minReadTemp = -10;
float maxReadTemp = 40;
float targetTempOffset = 6;
float movingTempOffset = 1.5;
int blurRange = 2;
float interpolationSpeed = 0.35;
float bloomBurst = 2;
bool isI2CValid = true;
bool hasI2C = true;
bool sensorReady = false;
float sensorStartDelay = 3;
float sensorStartTime = 0;

int SENSOR_WIDTH = 8;
int SENSOR_HEIGHT = 8;
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
float SENSOR_FPS = 10;
float RENDER_FPS = 30;

int i2c;

#define LED_PANEL_SIZE 32
#define LED_PANEL_COUNT 6
#define LED_PIXELS_WIDTH LED_PANEL_SIZE * 3
#define LED_PIXELS_HEIGHT LED_PANEL_SIZE * 3
#define LED_PIXEL_COUNT LED_PIXELS_WIDTH * LED_PIXELS_HEIGHT
#define LED_OUTPUT_WIDTH LED_PANEL_COUNT * LED_PANEL_SIZE

bool remapPixels (int& x, int& y) {
  int width = LED_PIXELS_WIDTH;
  int height = LED_PIXELS_HEIGHT;

  // now we remap this pixel to the new space
  int nx = x;
  int ny = y;

  // find out which row we are on
  int row;
  if (y >= 0 && y < 32) row = 0;
  else if (y >= 32 && y < 64) row = 1;
  else if (y >= 64 && y < 96) row = 2;
  else return false;

  if (row == 0) {
    // outside of top row bounds
    if (x < 32 || x >= 64) return false;
    // center top row of pixels
    nx -= 32;
  } else if (row == 1) {
    // outside of middle row bounds
    if (x < 16 || x >= 80) return false;
    // remap the Y value to the single row format
    ny -= 32;

    if (x >= 16 && x < 48) {
      // middle row, left panel...
      // third panel in the chain
      
      // remove border
      nx -= 16;
      // now offset & flip horizontally to correct position
      nx = 64 + (32 - nx - 1);
    } else {
      // don't need to do anything here,
      // srcX = (48..80) - border = 32..64
      // dstX = 32..64

      // remove border
      nx -= 16;
      // remove first panel
      nx -= 32;
      // offset and flip
      nx = 32 + (32 - nx - 1);
    }

    // flip the Y value
    ny = 32 - ny - 1;
  } else {
    // outside of bounds
    if (x < 0 || x >= 96) return false;

    // remap Y value to single row
    ny -= 64;
    // move to last 3 panels
    nx += (3 * 32);
  }

  // apply remap
  x = nx;
  y = ny;
  return true;
}

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

bool writeI2C () {
  try {
    // enter normal mode
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_PCTL, (unsigned int)0) < 0) {
      printf("Error entering normal mode on I2C\n");
      return false;
    }
    // initial reset
    int initialReset = AMG88xx_INITIAL_RESET & ((1 << 8) - 1);
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_RST, (unsigned int)(initialReset)) < 0) {
      printf("Error sending initial reset on I2C\n");
      return false;
    }
    // disable interrupt mode
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_INTC, (unsigned int)0) < 0) {
      printf("Error disabling interrupt mode on I2C\n");
      return false;
    }
    // set FPS to 10
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_FPSC, (unsigned int)0) < 0) {
      printf("Error setting FPS on I2C\n");
      return false;
    }
  } catch (const std::exception& err) {
    printf("Could not open I2C connection...\n");
    return false;
  }
  return true;
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

void resetFPS () {
  if (!hasI2C) return;
  try {
    // enter normal mode
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_PCTL, (unsigned int)0) < 0) {
      printf("Error entering normal mode on I2C\n");
      return;
    }
    // set FPS to 10
    if (wiringPiI2CWriteReg8(i2c, AMG88xx_FPSC, (unsigned int)0) < 0) {
      printf("Error setting FPS on I2C\n");
      return;
    }
  } catch (const std::exception& err) {
    printf("Could not open I2C connection...\n");
    return;
  }
}

void readPixels (float *buf) {
  // float converted;
	// uint8_t bytesToRead = min(size << 1, AMG88xx_PIXEL_ARRAY_SIZE << 1);
	// uint8_t rawArray[bytesToRead];
	// this->read(AMG88xx_PIXEL_OFFSET, rawArray, bytesToRead);
	bool isInvalid = false;
  // float avg = 0;
  // printf("thermistor: %f   ", readThermistor());
  float thermistor = abs(readThermistor()) / 2.0;
  
	for(int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++){
    int x = i % SENSOR_WIDTH;
    int y = i / SENSOR_WIDTH;
    // int srcIndex = i;
    y = SENSOR_HEIGHT - y - 1;
    int srcIndex = (SENSOR_HEIGHT - x - 1) * SENSOR_WIDTH + y;
    uint16_t raw = wiringPiI2CReadReg16(i2c, AMG88xx_PIXEL_OFFSET + (srcIndex << 1));
    
    // float converted = ((float)raw);
    // if (converted > 2047) converted = converted - 4097.0f;
    // converted *= AMG88xx_PIXEL_TEMP_CONVERSION;

    float converted = signedMag12ToFloat(raw) * AMG88xx_PIXEL_TEMP_CONVERSION;
    if (converted >= CLEAN_RANGE_MAX || converted <= CLEAN_RANGE_MIN) {
      isInvalid = true;
      // printf("I2C is invalid %f\n", converted);
      converted = thermistor;
    }
    // avg += converted;
    // converted = clamp(converted, -minReadTemp, maxReadTemp);
    
    // x = SENSOR_WIDTH - x - 1;
    // y = SENSOR_HEIGHT - y - 1;
    // int dstIndex = x + (y * SENSOR_WIDTH);

// data[];

    buf[i] = converted;
    // printf("%f ", buf[i]);
  }
  // avg /= (float)AMG88xx_PIXEL_ARRAY_SIZE;
  // printf("thermister: %f, pixel average: %f\n", readThermistor(), avg);

  // once we exit a clean state, expect it to always be dirty
  if (isI2CValid && isInvalid) {
    // printf("I2C is no longer valid!\n");
    // isI2CValid = false;
  }
  // printf("\n");
  
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
  }
};

class TemperatureView {
public:
  // the raw 8x8 temperatures
  float temperatures[AMG88xx_PIXEL_ARRAY_SIZE];

  // tweened 8x8 temperatures
  float interpolatedTemperatures[AMG88xx_PIXEL_ARRAY_SIZE];
  float tempBloomTemperatures[AMG88xx_PIXEL_ARRAY_SIZE];
  float bloomedTemperatures[AMG88xx_PIXEL_ARRAY_SIZE];

  std::deque<float> movingAverages;
  float movingAverageTemp = MINTEMP;
  float currentAverage = MINTEMP;

  float newAverage = MINTEMP;
  float periodTime = 30;
  int averagePeriod = (int)(periodTime * SENSOR_FPS);
  float newMovingAverageTemp = MINTEMP;
  float warmth = 0;
  float newWarmth = 0;
  float warmthAmbient = 0.75;
  float warmingFactor = 0.1;
  float warmingSpeed = 0.01;

  TemperatureView () {
    // clear temperatures
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      temperatures[i] = MINTEMP;
      interpolatedTemperatures[i] = MINTEMP;
      tempBloomTemperatures[i] = 0;
      bloomedTemperatures[i] = 0;
    }
  }

  float getValueAtPixel (int x, int y) {
    return bloomedTemperatures[y * SENSOR_WIDTH + x];
    // float temp = interpolatedTemperatures[(y * SENSOR_WIDTH) + x];
    // return unlerpClamp(movingAverageTemp + movingTempOffset, targetTemp, temp);
  }

  float getWarmthValue () {
    return warmth * warmthAmbient;
  }

  void update () {
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

  void blur (bool horizontal, int range) {
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      tempBloomTemperatures[i] = bloomedTemperatures[i];
    }

    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      int count = 0;
      float sum = 0;

      int x = i % SENSOR_WIDTH;
      int y = i / SENSOR_WIDTH;

      int lineIndex = horizontal ? x : y;
      int minVal = max(0, lineIndex - range);
      int maxVal = min(SENSOR_WIDTH - 1, lineIndex + range);
      for (int j = minVal; j <= maxVal; j++) {
        int nx = horizontal ? j : x;
        int ny = horizontal ? y : j;
        sum += bloomedTemperatures[nx + (ny * SENSOR_WIDTH)] * bloomBurst;
        count++;
      }
      float finalColor = count == 0 ? 0 : sum / count;
      tempBloomTemperatures[i] = clamp(finalColor, 0, 1);
    }

    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      bloomedTemperatures[i] = tempBloomTemperatures[i];
    }
  }

  void interpolate () {
    movingAverageTemp = lerp(movingAverageTemp, newMovingAverageTemp, interpolationSpeed);
    currentAverage = lerp(currentAverage, newAverage, interpolationSpeed);

    newWarmth = 0;
    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      float f = lerp(interpolatedTemperatures[i], temperatures[i], interpolationSpeed);
      interpolatedTemperatures[i] = f;

      float unlerpedPixel = unlerpClamp(movingAverageTemp + movingTempOffset, movingAverageTemp + movingTempOffset + targetTempOffset, f);
      bloomedTemperatures[i] = unlerpedPixel;
      // int x = i % SENSOR_WIDTH;
      // int y = i / SENSOR_WIDTH;
      // x = SENSOR_WIDTH - x - 1;
      // int dstIndex = x + (y * SENSOR_WIDTH);
      
      // get total 
      float pixelWarmth = unlerpedPixel;
      newWarmth += pixelWarmth * warmingFactor;
    }
    newWarmth = min(1.0f, newWarmth);
    warmth = lerp(warmth, newWarmth, warmingSpeed);

    // bloom the pixels a bit
    if (blurRange > 0) {
      blur(true, blurRange);
      blur(false, blurRange);
    }
  }
};

// // Simple class that generates a rotating block on the screen.
// class TestView : public ThreadedCanvasManipulator {
// public:
//   ColorRGB *colorBlack = new ColorRGB(0, 0, 0);

//   TestView(Canvas *m) : ThreadedCanvasManipulator(m) {}

//   void Run () {
//     int width = 96;
//     int height = 96;
//     ColorRGB *fragColor = new ColorRGB(0, 0, 0);

//     canvas()->Fill(0, 0, 0);
//     printf("WidthHeight %d x %d\n", width, height);
//     for (int i = 0; i < 96; i++) {
//       int x = i;
//       int y = 33;
//       float px = x / (float)width;
//       int v = (int)(px*255);
//       // printf("Setting %dx%d\n", x, y);
//       if (remapPixels(x, y)) {
//         canvas()->SetPixel(x, y, v, 0, 0);
//       }
//     }
//   }
// };

// Simple class that generates a rotating block on the screen.
class SDFGen : public ThreadedCanvasManipulator {
public:
  typedef std::chrono::high_resolution_clock clock_;
  typedef std::chrono::duration<double, std::ratio<1> > second_;
  std::chrono::time_point<clock_> lastTime;
  Vec2 *tempVec = new Vec2(0, 0);

  ColorRGB *colorBlack = new ColorRGB(0, 0, 0);
  ColorRGB *colorWhite = new ColorRGB(255, 255, 255);
  ColorRGB *colorRed = new ColorRGB(255, 0, 0);

  ColorRGB *colorIdle0 = new ColorRGB(57, 183, 255);
  ColorRGB *colorIdle1 = new ColorRGB(30, 255, 195);
  ColorRGB *colorIdle2 = new ColorRGB(62, 50, 239); //57, 76, 255

  ColorRGB *colorDrama0 = new ColorRGB(70, 22, 244);//34, 1, 247

  ColorRGB *colorActive0 = new ColorRGB(255, 0, 0);
  ColorRGB *colorActive1 = new ColorRGB(249, 145, 10);
  ColorRGB *baseColor = new ColorRGB(0, 0, 0);
  ColorRGB *tempColor = new ColorRGB(0, 0, 0);

  double currentTime = 0.0;
  double sensorTime = 0.0;
  double fpsResetTime = 0;
  double fpsResetDelay = 1;
  double sensorInterval = 1.0 / RENDER_FPS;
  double frameTime = 0.0;
  double frameInterval = 1.0 / 30;
  double resetInterval = 60 * 2;
  double resetTime = 0;

  FastNoise noise; // Create a FastNoise object
  TemperatureView *sensor = new TemperatureView();

  SDFGen(Canvas *m) : ThreadedCanvasManipulator(m) {}

  void Run () {
    int width = 96;
    int height = 96;
    ColorRGB *fragColor = new ColorRGB(0, 0, 0);

    noise.SetNoiseType(FastNoise::Simplex); // Set the desired noise type

    // start time
    lastTime = clock_::now();

    while (running() && !interrupt_received) {
      // printf("thermistor %f, current average %f, moving average %f\n", readThermistor(), sensor->currentAverage, sensor->movingAverageTemp + movingTempOffset);
      std::chrono::time_point<clock_> newTime = clock_::now();
      double dt = std::chrono::duration_cast<second_>(newTime - lastTime).count();
      currentTime += dt;
      lastTime = newTime;
      sensorTime += dt;

      if (!sensorReady) {
        sensorStartTime += dt;
        if (sensorStartTime > sensorStartDelay) {
          // allow I2C and I2C resets
          sensorReady = true;
          printf("Sensor ready.\n");

          // also reset I2C at this time
          resetTime = 0;
          hasI2C = writeI2C();
          printf("Initial Reset %d\n", hasI2C);
        }
      }

      fpsResetTime += dt;
      if (sensorReady && fpsResetTime > fpsResetDelay) {
        fpsResetTime = 0;
        // printf("Reset FPS\n");
        resetFPS();
      }

      resetTime += dt;
      if (sensorReady && resetTime > resetInterval) {
        resetTime = 0;
        hasI2C = writeI2C();
        printf("Reset %d\n", hasI2C);
      }
      if (sensorReady && hasI2C && sensorTime > sensorInterval) {
        sensorTime = 0.0;
        // read into temperatures
        readPixels(sensor->temperatures);
        // update view
        sensor->update();
        // printf("Avg temp %f\n", sensor->currentAverage);
      }
      frameTime += dt;
      if (frameTime > frameInterval) {
        frameTime = 0.0;
        // usleep(15 * 1000);
        // interpolate toward the new rolling averages
        if (hasI2C) sensor->interpolate();
        // printf("temp %f\n", newAverage);

        float colorOffset = sinf(currentTime * 0.25) * 0.5 + 0.5;
        // choose secondary color based on slowly rotating offset
        baseColor->copy(colorIdle1);
        baseColor->lerp(colorIdle2, colorOffset);

        // slowly rotate to a more dramatic secondary color
        float coloroffset2 = sinf(currentTime * 0.1) * 0.5 + 0.5;
        coloroffset2 = smoothstep(0.0, 0.1, coloroffset2);
        baseColor->lerp(colorDrama0, coloroffset2);

        for (int i = 0; i < width * height; i++) {
          int x = i % width;
          int y = i / width;

          float px = (x / (float)width);
          float py = (y / (float)height);

          // flip image before rendering
          int dstX = x;
          int dstY = y;
          if (remapPixels(dstX, dstY)) {
            fragColor->copy(colorBlack);
            pixelShader(fragColor, x, y, px, py, (float)width, (float)height, i);
            fragColor->clampBytes();
            canvas()->SetPixel(dstX, dstY, fragColor->r, fragColor->g, fragColor->b);
          }
        }
      }
    }
  }

  void pixelShader (ColorRGB *fragColor, int x, int y, float xCoord, float yCoord, float width, float height, int index) {
    float aspect = width / height;

    int xRatio = (int)((SENSOR_WIDTH << 16) / width) + 1;
    int yRatio = (int)((SENSOR_HEIGHT << 16) / height) + 1;

    int srcX = ((x * xRatio) >> 16);
    int srcY = ((y * yRatio) >> 16);
    float interaction = (isI2CValid && hasI2C) ? sensor->getValueAtPixel(srcX, srcY) : 0;

    float zoom, speed;

    // mix secondary with primary using noise
    zoom = 40 * 2;
    speed = 25 * 0.5;
    float n2 = (noise.GetNoise(xCoord * zoom * aspect, yCoord * zoom, currentTime * speed) * 0.5 + 0.5);
    tempColor->copy(baseColor);
    tempColor->lerp(colorIdle0, n2);

    // mix color in with noise
    zoom = 80 * 2;
    speed = 35;
    float n1 = (noise.GetNoise(xCoord * zoom * aspect, yCoord * zoom, currentTime * speed) * 0.5 + 0.5);
    n1 = clamp(powf(n1, 1.5), 0, 1);
    fragColor->lerp(tempColor, n1);

    // apply interaction color
    // fragColor->copy(colorBlack);
    if (sensorReady && isI2CValid && hasI2C) {
      // now mix in active color
      tempColor->copy(colorActive0);
      tempColor->lerp(colorActive1, n1);

      fragColor->lerp(tempColor, interaction);
      // float v = interaction;
      // fragColor->setRGBFloat(v, v, v);
      // move toward overall warmth color
      fragColor->lerp(tempColor, sensor->getWarmthValue() * 0.35);
      // fragColor->setRGBFloat(interaction, interaction, interaction);
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
  if (hasI2C) {
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
  }
  // if (hasI2C) hasI2C = writeI2C();

  RGBMatrix::Options matrix_options;
  rgb_matrix::RuntimeOptions runtime_opt;

  // These are the defaults when no command-line flags are given.
  matrix_options.rows = 32;
  matrix_options.chain_length = 6;
  matrix_options.parallel = 1;

  // First things first: extract the command line flags that contain
  // relevant matrix options.
  if (!ParseOptionsFromFlags(&argc, &argv, &matrix_options, &runtime_opt)) {
    return usage(argv[0]);
  }

  RGBMatrix *matrix = CreateMatrixFromOptions(matrix_options, runtime_opt);
  if (matrix == NULL)
    return 1;

  if (matrix_options.chain_length > 3) {
    printf("Using LUMOS arrangement...\n");
    matrix->ApplyStaticTransformer(LumosArrangementTransformer());
  }

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

  if (hasI2C) {
    // wiringPi2CClose(i2c);
  }

  // Stop image generating thread. The delete triggers
  delete image_gen;
  delete canvas;

  printf("\%s. Exiting.\n",
         interrupt_received ? "Received CTRL-C" : "Timeout reached");
  return 0;
}
