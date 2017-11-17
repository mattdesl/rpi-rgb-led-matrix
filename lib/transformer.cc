// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2014 Henner Zeller <h.zeller@acm.org>
// Copyright (C) 2015 Christoph Friedrich <christoph.friedrich@vonaffenfels.de>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#include <assert.h>
#include <stdio.h>

#include "transformer.h"

namespace rgb_matrix {

/*****************************/
/* Rotate Transformer Canvas */
/*****************************/
class RotateTransformer::TransformCanvas : public Canvas {
public:
  TransformCanvas(int angle);

  void SetDelegatee(Canvas* delegatee);
  void SetAngle(int angle);

  virtual int width() const;
  virtual int height() const;
  virtual void SetPixel(int x, int y, uint8_t red, uint8_t green, uint8_t blue);
  virtual void Clear();
  virtual void Fill(uint8_t red, uint8_t green, uint8_t blue);

private:
  Canvas *delegatee_;
  int angle_;
};

RotateTransformer::TransformCanvas::TransformCanvas(int angle)
  : delegatee_(NULL) {
  SetAngle(angle);
}

void RotateTransformer::TransformCanvas::SetDelegatee(Canvas* delegatee) {
  delegatee_ = delegatee;
}

void RotateTransformer::TransformCanvas::SetPixel(int x, int y, uint8_t red, uint8_t green, uint8_t blue) {
  switch (angle_) {
  case 0:
    delegatee_->SetPixel(x, y, red, green, blue);
    break;
  case 90:
    delegatee_->SetPixel(delegatee_->width() - y - 1, x,
                         red, green, blue);
    break;
  case 180:
    delegatee_->SetPixel(delegatee_->width() - x - 1,
                         delegatee_->height() - y - 1,
                         red, green, blue);
    break;
  case 270:
    delegatee_->SetPixel(y, delegatee_->height() - x - 1, red, green, blue);
    break;
  }
}

int RotateTransformer::TransformCanvas::width() const {
  return (angle_ % 180 == 0) ? delegatee_->width() : delegatee_->height();
}

int RotateTransformer::TransformCanvas::height() const {
  return (angle_ % 180 == 0) ? delegatee_->height() : delegatee_->width();
}

void RotateTransformer::TransformCanvas::Clear() {
  delegatee_->Clear();
}

void RotateTransformer::TransformCanvas::Fill(uint8_t red, uint8_t green, uint8_t blue) {
  delegatee_->Fill(red, green, blue);
}

void RotateTransformer::TransformCanvas::SetAngle(int angle) {
  assert(angle % 90 == 0);  // We currenlty enforce that for more pretty output
  angle_ = (angle + 360) % 360;
}

/**********************/
/* Rotate Transformer */
/**********************/
RotateTransformer::RotateTransformer(int angle)
  : angle_(angle), canvas_(new TransformCanvas(angle)) {
}

RotateTransformer::~RotateTransformer() {
  delete canvas_;
}

Canvas *RotateTransformer::Transform(Canvas *output) {
  assert(output != NULL);

  canvas_->SetDelegatee(output);
  return canvas_;
}

void RotateTransformer::SetAngle(int angle) {
  canvas_->SetAngle(angle);
  angle_ = angle;
}

/**********************/
/* Linked Transformer */
/**********************/
void LinkedTransformer::AddTransformer(CanvasTransformer *transformer) {
  list_.push_back(transformer);
}

void LinkedTransformer::AddTransformer(List transformer_list) {
  list_.insert(list_.end(), transformer_list.begin(), transformer_list.end());
}
void LinkedTransformer::SetTransformer(List transformer_list) {
  list_ = transformer_list;
}

Canvas *LinkedTransformer::Transform(Canvas *output) {
  for (size_t i = 0; i < list_.size(); ++i) {
    output = list_[i]->Transform(output);
  }

  return output;
}

void LinkedTransformer::DeleteTransformers() {
  for (size_t i = 0; i < list_.size(); ++i) {
    delete list_[i];
  }
  list_.clear();
}

// LUMOS Transformer
class LumosArrangementTransformer::TransformCanvas : public Canvas {
public:
  TransformCanvas() : delegatee_(NULL) {}

  void SetDelegatee(Canvas* delegatee);

  virtual void Clear();
  virtual void Fill(uint8_t red, uint8_t green, uint8_t blue);
  virtual int width() const { return width_; }
  virtual int height() const { return height_; }
  virtual void SetPixel(int x, int y, uint8_t red, uint8_t green, uint8_t blue);

private:
  int panelWidth = 32;
  int panelHeight = 32;
  int panelsAcross = 6;
  int panelsHigh = 1;

  int width_ = panelWidth * panelsAcross;
  int height_ = panelHeight * panelsHigh;

  Canvas *delegatee_;
};

void LumosArrangementTransformer::TransformCanvas::SetDelegatee(Canvas* delegatee) {
  delegatee_ = delegatee;
}

void LumosArrangementTransformer::TransformCanvas::Clear() {
  delegatee_->Clear();
}

void LumosArrangementTransformer::TransformCanvas::Fill(
  uint8_t red, uint8_t green, uint8_t blue) {
  delegatee_->Fill(red, green, blue);
}

void LumosArrangementTransformer::TransformCanvas::SetPixel(
  int x, int y, uint8_t red, uint8_t green, uint8_t blue) {
  
  int nx = x;
  int ny = y;
  int panel = (int)((float)x / panelWidth);
  int panelX = x % panelWidth;
  int row;
  
  // if (panel == 0) row = 0;
  // else if (panel >= 2 && panel <= 1) row = 1;
  // else if (panel >= 3 && panel <= 5) row = 2;
  // else return;

  // second panel
  // if (x >= 32 && x < 64 && y >= 0 && y < 32) {
  //   ny += 2;
  // }
  // printf("Incoming %dx%d - row %d, panel %d\n", x, y, row, panel);
  if (row == 0) {
    // don't need to do anything for top panel
    
  } else if (row == 1) {
    // ny = (panelHeight * row) + ny;
    // nx = panelX
    
  } else {
    // we're on the last row
    // figure out which we are from leftmost=0 to rightmost=2
    // int panelIndex = 2 - (5 - panel);

    // printf("Incoming %dx%d - panel %d\n", ox, y, panel);

    // now figure out how much we offset by x
    // nx = panelX * panelIndex;
    // ny = (panelHeight * row) + ny;
    // offset by row
    // ny += panelHeight * row;
    // printf("Incoming %dx%d - outgoing %dx%d - panel %d, %d\n", x, y, nx, ny, panel, panelX);
  }

  // int row, column;
  // if (y >= 0 && y < 32) column = 0;
  // else if (y >= 32 && y < 64) column = 1;
  // else if (y >= 64 && y < 96) column = 2;
  // else return;

  // int nx = x;
  // int ny = y;
  // if (column == 2) {
    
  //   // outside of bounds
  //   if (x < 0 || x >= 96) return;

  //   // remap Y value to single row
  //   ny -= 64;
  //   // move to last 3 panels
  //   nx += (3 * 32);
  // printf("Incoming %dx%d, outgoing %dx%d\n", x, y, nx, ny);
  // }

  // if (column == 0) {
  //   // outside of top row bounds
  //   if (x < 32 || x >= 64) return;
  //   // center top row of pixels
  //   nx -= 32;
  // } else if (column == 1) {
    // // outside of middle row bounds
    // if (x < 16 || x >= 80) return;
    // // remap the Y value to the single row format
    // ny -= 32;

    // if (x >= 16 && x < 48) {
    //   // middle row, left panel...
    //   // third panel in the chain
      
    //   // remove border
    //   nx -= 16;
    //   // now offset & flip horizontally to correct position
    //   nx = 64 + (32 - nx - 1);
    // } else {
    //   // don't need to do anything here,
    //   // srcX = (48..80) - border = 32..64
    //   // dstX = 32..64

    //   // remove border
    //   nx -= 16;
    //   // remove first panel
    //   nx -= 32;
    //   // offset and flip
    //   nx = 32 + (32 - nx - 1);
    // }

    // // flip the Y value
    // ny = 32 - ny - 1;
  //   // printf("Incoming %dx%d, outgoing %dx%d\n", x, y, nx, ny);
    
  // } else if (column == 2) {
    // // outside of bounds
    // if (x < 0 || x >= 96) return;

    // // remap Y value to single row
    // ny -= 64;
    // // move to last 3 panels
    // nx += (3 * 32);
  // }

  // offset by one for some reason
  delegatee_->SetPixel(nx, ny, red, green, blue);
}

LumosArrangementTransformer::LumosArrangementTransformer()
  : canvas_(new TransformCanvas()) {
}

LumosArrangementTransformer::~LumosArrangementTransformer() {
  delete canvas_;
}

Canvas *LumosArrangementTransformer::Transform(Canvas *output) {
  assert(output != NULL);

  canvas_->SetDelegatee(output);
  return canvas_;
}

// U-Arrangement Transformer.
class UArrangementTransformer::TransformCanvas : public Canvas {
public:
  TransformCanvas(int parallel) : parallel_(parallel), delegatee_(NULL) {}

  void SetDelegatee(Canvas* delegatee);

  virtual void Clear();
  virtual void Fill(uint8_t red, uint8_t green, uint8_t blue);
  virtual int width() const { return width_; }
  virtual int height() const { return height_; }
  virtual void SetPixel(int x, int y, uint8_t red, uint8_t green, uint8_t blue);

private:
  const int parallel_;
  int width_;
  int height_;
  int panel_height_;
  Canvas *delegatee_;
};

void UArrangementTransformer::TransformCanvas::SetDelegatee(Canvas* delegatee) {
  delegatee_ = delegatee;
  width_ = (delegatee->width() / 64) * 32;   // Div in middle at 32px boundary
  height_ = 2 * delegatee->height();
  if (delegatee->width() % 64 != 0) {
    fprintf(stderr, "An U-arrangement would need an even number of panels "
            "unless you can fold one in the middle...\n");
  }
  if (delegatee->height() % parallel_ != 0) {
    fprintf(stderr, "For parallel=%d we would expect the height=%d to be "
            "divisible by %d ??\n", parallel_, delegatee->height(), parallel_);
    assert(false);
  }
  panel_height_ = delegatee->height() / parallel_;
}

void UArrangementTransformer::TransformCanvas::Clear() {
  delegatee_->Clear();
}

void UArrangementTransformer::TransformCanvas::Fill(
  uint8_t red, uint8_t green, uint8_t blue) {
  delegatee_->Fill(red, green, blue);
}

void UArrangementTransformer::TransformCanvas::SetPixel(
  int x, int y, uint8_t red, uint8_t green, uint8_t blue) {
  if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
  const int slab_height = 2*panel_height_;   // one folded u-shape
  const int base_y = (y / slab_height) * panel_height_;
  y %= slab_height;
  if (y < panel_height_) {
    x += delegatee_->width() / 2;
  } else {
    x = width_ - x - 1;
    y = slab_height - y - 1;
  }
  delegatee_->SetPixel(x, base_y + y, red, green, blue);
}

UArrangementTransformer::UArrangementTransformer(int parallel)
  : canvas_(new TransformCanvas(parallel)) {
  assert(parallel > 0);
}

UArrangementTransformer::~UArrangementTransformer() {
  delete canvas_;
}

Canvas *UArrangementTransformer::Transform(Canvas *output) {
  assert(output != NULL);

  canvas_->SetDelegatee(output);
  return canvas_;
}

// Legacly LargeSquare64x64Transformer: uses the UArrangementTransformer, but
// does things so that it looks the same as before.
LargeSquare64x64Transformer::LargeSquare64x64Transformer()
  : arrange_(1), rotated_(180) { }
Canvas *LargeSquare64x64Transformer::Transform(Canvas *output) {
  return rotated_.Transform(arrange_.Transform(output));
}
} // namespace rgb_matrix
