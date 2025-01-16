/**
 *
 *  \file
 *  \brief      Sequence Classes header
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
 */


#ifndef CLEARPATH_HARDWARE_INTERFACES__LIGHTING__SEQUENCE_HPP_
#define CLEARPATH_HARDWARE_INTERFACES__LIGHTING__SEQUENCE_HPP_

#include <vector>

#include "clearpath_hardware_interfaces/lighting/color.hpp"
#include "clearpath_hardware_interfaces/lighting/platform.hpp"
#include "clearpath_platform_msgs/msg/lights.hpp"
#include "clearpath_platform_msgs/msg/rgb.hpp"

namespace clearpath_lighting
{

// State of each RGB light at an instance
typedef std::vector<ColorHSV> LightingState;
// Vector of N color sequences. N = number of LEDs
typedef std::vector<std::vector<ColorHSV>> LightingSequence;

enum LightingPattern
{
  Solid,
  Blinking,
  Pulsing
};

class Sequence
{

public:
  Sequence();

  clearpath_platform_msgs::msg::Lights getLightsMsg();
  void reset();
  static LightingState fillLightingState(ColorHSV color, clearpath_lighting::Platform platform);
  static LightingState fillFrontRearLightingState(ColorHSV front_color, ColorHSV rear_color, clearpath_lighting::Platform platform);
  static LightingState fillLeftRightLightingState(ColorHSV left_color, ColorHSV right_color, clearpath_lighting::Platform platform);
  static LightingState fillOppositeCornerLightingState(ColorHSV front_left_color, ColorHSV front_right_color, clearpath_lighting::Platform platform);
  const LightingSequence& getSequence() const;
  void setSequence(LightingSequence sequence);

  uint16_t getNumStates() const;
  void setNumStates(const uint16_t num_states);

  Sequence operator+(Sequence const& other)
  {
    Sequence s;
    LightingSequence ls, other_ls;

    ls = sequence_;
    other_ls = other.getSequence();

    for (std::size_t i = 0; i < sequence_.size(); i++)
    {
      ls.at(i).insert(ls.at(i).end(), other_ls.at(i).begin(), other_ls.at(i).end());
    }

    s.setSequence(ls);
    s.setNumStates(num_states_ + other.getNumStates());

    return s;
  }

  bool operator==(Sequence const& other)
  {
    LightingSequence ls, other_ls;

    ls = sequence_;
    other_ls = other.getSequence();
    
    for (std::size_t i = 0; i < sequence_.size(); i++)
    {
      if (ls.at(i) != other_ls.at(i))
      {
        return false;
      }
    }

    return true;
  }

  bool operator!=(Sequence const& other)
  {
    return !(sequence_ == other.getSequence());
  }

protected:
  LightingSequence sequence_;
  uint16_t current_state_, num_states_;
};

class SolidSequence : public Sequence
{
public:
  SolidSequence(const LightingState state);
};

class BlinkSequence : public Sequence
{
public:
  BlinkSequence(const LightingState first_state,
                const LightingState second_state,
                uint32_t steps,
                double duty_cycle);
};

class PulseSequence : public Sequence
{
public:
  PulseSequence(const LightingState first_state,
                const LightingState last_state,
                uint32_t steps);
};

}  // namespace clearpath_lighting

#endif  // CLEARPATH_HARDWARE_INTERFACES__LIGHTING__SEQUENCE_HPP_
