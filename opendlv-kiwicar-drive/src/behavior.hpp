/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BEHAVIOR
#define BEHAVIOR

#include <mutex>
#include <random> 
#include "opendlv-message-standard.hpp"
#include "custom-messages.hpp"


#include "cluon-complete.hpp"

class Behavior
{
private:
  Behavior(Behavior const &) = delete;
  Behavior(Behavior &&) = delete;
  Behavior &operator=(Behavior const &) = delete;
  Behavior &operator=(Behavior &&) = delete;

public:
  Behavior() noexcept;
  ~Behavior() = default;

public:
  void setFrontUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
  void setRearUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
  void setLeftIr(opendlv::proxy::DistanceReading const &) noexcept;
  void setRightIr(opendlv::proxy::DistanceReading const &) noexcept;
  void setDetectionBoundingBox(opendlv::logic::perception::DetectionBoundingBox const &) noexcept;
  void addDetectionBoundingBox(opendlv::logic::perception::DetectionBoundingBox const &) noexcept;
  void clearDetectionBoundingBoxes() noexcept;
  void setRecievingCameraData(bool) noexcept;
  void setFrameId(uint32_t) noexcept;
  void setRandomAngle(float) noexcept;
  void step(cluon::OD4Session &, float, float, float, float, float, float, bool,int, float) noexcept;


private:
  opendlv::proxy::DistanceReading m_frontUltrasonicReading;
  opendlv::proxy::DistanceReading m_rearUltrasonicReading;
  opendlv::proxy::DistanceReading m_leftIrReading;
  opendlv::proxy::DistanceReading m_rightIrReading;
  opendlv::logic::perception::DetectionBoundingBox m_detectionBoundingBox;
  std::vector<opendlv::logic::perception::DetectionBoundingBox> m_detectionBoundingBoxes;
  bool m_recievingData;
  uint32_t m_frameId;
  float m_lastRandomAngle;
  // srand (static_cast <unsigned> (time(0)));

  std::mutex m_frontUltrasonicReadingMutex;
  std::mutex m_rearUltrasonicReadingMutex;
  std::mutex m_leftIrReadingMutex;
  std::mutex m_rightIrReadingMutex;
  std::mutex m_detectionBoundingBoxMutex;
  std::mutex m_detectionBoundingBoxesMutex;
  std::mutex m_recievingDataMutex;
  std::mutex m_frameIdMutex;
  std::mutex m_lastRandomAngleMutex;

};

#endif
