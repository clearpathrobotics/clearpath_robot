/**
 *
 *  \file
 *  \brief      Color Class Header
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2023, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 * Adapted from https://gist.github.com/borgel/d9a8bc11aeb5e0005d8320026c46f6f7
 */

#ifndef CLEARPATH_HARDWARE_INTERFACES__LIGHTING__COLOR_HPP_
#define CLEARPATH_HARDWARE_INTERFACES__LIGHTING__COLOR_HPP_

#include <vector>
#include <stdint.h>
#include "clearpath_platform_msgs/msg/rgb.hpp"

namespace clearpath_lighting
{

struct hsv_t {
  double h;
  double s;
  double v;

  hsv_t(double h_, double s_, double v_)
  {
    h = h_;
    s = s_;
    v = v_;
  }

  hsv_t()
  {
    h = 0.0;
    s = 0.0;
    v = 0.0;
  }

  bool operator==(hsv_t const& other)
  {
    return (h == other.h && s == other.s && v == other.v);
  }

  bool operator!=(hsv_t const& other)
  {
    return !(*this == other);
  }
};

static const hsv_t COLOR_RED = hsv_t(0.0, 100.0, 37.5);
static const hsv_t COLOR_RED_DIM = hsv_t(0.0, 100.0, 18.75);
static const hsv_t COLOR_MAGENTA = hsv_t(300.0, 100.0, 25.0);
static const hsv_t COLOR_BLUE = hsv_t(240.0, 100.0, 25.0);
static const hsv_t COLOR_BLUE_DIM = hsv_t(240.0, 100.0, 12.5);
static const hsv_t COLOR_CYAN = hsv_t(180.0, 100.0, 25.0);
static const hsv_t COLOR_GREEN = hsv_t(120.0, 100.0, 25.0);
static const hsv_t COLOR_GREEN_DIM = hsv_t(120.0, 100.0, 12.5);
static const hsv_t COLOR_YELLOW = hsv_t(60.0, 100.0, 25.0);
static const hsv_t COLOR_ORANGE = hsv_t(30.0, 100.0, 25.0);
static const hsv_t COLOR_ORANGE_DARK = hsv_t(30.0, 100.0, 1.0);
static const hsv_t COLOR_WHITE = hsv_t(0.0, 0.0, 25.0);
static const hsv_t COLOR_WHITE_DIM = hsv_t(0.0, 0.0, 12.5);
static const hsv_t COLOR_BLACK = hsv_t(0.0, 0.0, 0.0);


class ColorHSV
{

public:
  ColorHSV(hsv_t hsv);
  static std::vector<ColorHSV> fade(ColorHSV start, ColorHSV end, uint32_t steps);
  clearpath_platform_msgs::msg::RGB getRgbMsg();
  double h() const { return hsv_.h; };
  double s() const { return hsv_.s; };
  double v() const { return hsv_.v; };

  void setH(const double h)
  {
    hsv_.h = h;
  }

  void setS(const double s)
  {
    hsv_.s = s;
  }

  void setV(const double v)
  {
    hsv_.v = v;
  }

  bool operator==(const ColorHSV other) const
  {
    return (h() == other.h() && s() == other.s() && v() == other.v());
  }

  bool operator!=(const ColorHSV other) const
  {
    return !(*this == other);
  }

private:
  hsv_t hsv_;
};

}

#endif  // CLEARPATH_HARDWARE_INTERFACES__LIGHTING__COLOR_HPP_
