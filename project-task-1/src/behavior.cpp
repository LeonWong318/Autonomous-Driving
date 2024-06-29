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

#include "behavior.hpp"
#include "cluon-complete.hpp"
#include <iostream>

float const PI = 3.14159265358979323846f;
float const toRad = PI / 180.0f;
int const CLASS_KIWICAR = 0;
int const CLASS_CONE_BLUE = 1;
int const CLASS_CONE_YELLOW = 2;
int const CLASS_PAPER_BLUE = 3;
int const CLASS_POSTIT_GREEN = 4;

Behavior::Behavior() noexcept
    : m_frontUltrasonicReading{}, m_rearUltrasonicReading{}, m_leftIrReading{},
      m_rightIrReading{}, m_detectionBoundingBox{}, m_detectionBoundingBoxes{},
      m_recievingData{}, m_frameId{}, m_angle{}, m_frontUltrasonicReadingMutex{},
      m_rearUltrasonicReadingMutex{}, m_leftIrReadingMutex{},
      m_rightIrReadingMutex{}, m_detectionBoundingBoxMutex{},
      m_detectionBoundingBoxesMutex(), m_recievingDataMutex{}, m_frameIdMutex(), m_angleMutex{}
{
}

void Behavior::setFrontUltrasonic(
    opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(
    opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(
    opendlv::proxy::DistanceReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(
    opendlv::proxy::DistanceReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

void Behavior::setRecievingCameraData(bool recievingData) noexcept
{
  std::lock_guard<std::mutex> lock(m_recievingDataMutex);
  m_recievingData = recievingData;
}

void Behavior::setFrameId(uint32_t frameId) noexcept
{
  std::lock_guard<std::mutex> lock(m_frameIdMutex);
  m_frameId = frameId;
}

void Behavior::clearDetectionBoundingBoxes() noexcept
{
  std::lock_guard<std::mutex> lock(m_detectionBoundingBoxesMutex);
  m_detectionBoundingBoxes.clear();
}

void Behavior::addDetectionBoundingBox(
    opendlv::logic::perception::DetectionBoundingBox const &boundingBox) noexcept
{
  std::lock_guard<std::mutex> lock(m_detectionBoundingBoxesMutex);
  m_detectionBoundingBoxes.push_back(boundingBox);
}

void Behavior::setAngle(float angle) noexcept
{
  std::lock_guard<std::mutex> lock(m_angleMutex);
  m_angle = angle;
}

void Behavior::step(
    cluon::OD4Session &od4,
    float baseSpeed,
    int convertAngle,
    bool verbose,
    float xThresLow,
    float xThresHigh,
    float yThresLow,
    float yThresHigh) noexcept
{
  bool recievingData;
  float lastAngle;
  {
    std::lock_guard<std::mutex> lock6(m_recievingDataMutex);
    std::lock_guard<std::mutex> lock8(m_angleMutex);
    recievingData = m_recievingData;
    lastAngle = m_angle;
  }


  opendlv::proxy::PedalPositionRequest pedalPositionRequest;
  opendlv::proxy::GroundSteeringRequest groundSteeringRequest;



  if (recievingData == true)
  {
    if (verbose)
    {
      std::cout << "Recieving data" << std::endl;
    }
    pedalPositionRequest.position(baseSpeed);
    groundSteeringRequest.groundSteering(lastAngle);
    od4.send(pedalPositionRequest);
    od4.send(groundSteeringRequest);
    return;
  }
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::DistanceReading leftIrReading;
  opendlv::proxy::DistanceReading rightIrReading;
  std::vector<opendlv::logic::perception::DetectionBoundingBox>
      detectionBoundingBoxes;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);
    std::lock_guard<std::mutex> lock5(m_detectionBoundingBoxesMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
    detectionBoundingBoxes = m_detectionBoundingBoxes;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = leftIrReading.distance();
  double rightDistance = rightIrReading.distance();
  float xBlue, x2Blue, xYellow, x2Yellow;
  int boundingBoxIndexBlue = -1, boundingBoxIndexYellow = -1;
  float steerAngle, centerxBlue, centerxYellow;
  int closestYellowIndex = -1;
  // float angle;

  if (verbose)
  {
    std::cout << "----------------------" << std::endl;
    std::cout << "\t" << frontDistance << "\t" << std::endl;
    std::cout << leftDistance << " \t" << rightDistance << std::endl;
    std::cout << "\t" << rearDistance << "\t" << std::endl;
  }
  // We see nothing, go straight
  if (detectionBoundingBoxes.size() == 0)
  {
    steerAngle = 0.0f;
    if (verbose)
    {
      std::cout << "No boxes: Go straight" << std::endl;
    }
  }
  else
  {

    for (size_t i = 0; i < detectionBoundingBoxes.size(); i++)
    {
      if (detectionBoundingBoxes[i].detectionId() == CLASS_CONE_BLUE) // CLASS_KIWICAR
      {
        // std::cout << "Blue" << "X: " << detectionBoundingBoxes[i].x() << "Y: " << detectionBoundingBoxes[i].y()
        // << "W: " << detectionBoundingBoxes[i].width() << "H: " << detectionBoundingBoxes[i].height() << std::endl;
        // float area = detectionBoundingBoxes[i].width() * detectionBoundingBoxes[i].height();
        if (boundingBoxIndexBlue == -1)
        {
          boundingBoxIndexBlue = i;
        }
        else if (detectionBoundingBoxes[i].x() < detectionBoundingBoxes[boundingBoxIndexBlue].x())
        {
          boundingBoxIndexBlue = i;
        }
      }
      if (detectionBoundingBoxes[i].detectionId() == CLASS_CONE_YELLOW) // CLASS_KIWICAR
      {
        // std::cout << "Yellow" << "X: " << detectionBoundingBoxes[i].x() << "Y: " << detectionBoundingBoxes[i].y()
        // << "W: " << detectionBoundingBoxes[i].width() << "H: " << detectionBoundingBoxes[i].height() << std::endl;
        // float area = detectionBoundingBoxes[i].width() * detectionBoundingBoxes[i].height();
        if (boundingBoxIndexYellow == -1)
        {
          boundingBoxIndexYellow = i;
          closestYellowIndex = i;
        } else {
          if (detectionBoundingBoxes[i].x() < detectionBoundingBoxes[boundingBoxIndexYellow].x())
          {
            boundingBoxIndexYellow = i;
          }
          if (detectionBoundingBoxes[i].y() > detectionBoundingBoxes[closestYellowIndex].y()){
            closestYellowIndex = i;
          }
        }
        
      }
    }

    steerAngle = 0.0f;

    

    if (boundingBoxIndexBlue != -1)
    {
      // blue boxes - turn
      xBlue = detectionBoundingBoxes[boundingBoxIndexBlue].x();
      x2Blue = detectionBoundingBoxes[boundingBoxIndexBlue].x() + detectionBoundingBoxes[boundingBoxIndexBlue].width();
      centerxBlue = (xBlue + x2Blue) / 2.0f;

      if (centerxBlue < xThresLow)
      {
        steerAngle = 0.5f * convertAngle;
      }
      else if (centerxBlue < xThresHigh)
      {
        steerAngle = 0.45f * convertAngle;
      }
      else
      {
        steerAngle = 0.0f;
      }
    }

    if (boundingBoxIndexYellow != -1)
    {
      // yellow boxes - turn
      xYellow = detectionBoundingBoxes[boundingBoxIndexYellow].x();
      x2Yellow = detectionBoundingBoxes[boundingBoxIndexYellow].x() + detectionBoundingBoxes[boundingBoxIndexYellow].width();
      centerxYellow = (xYellow + x2Yellow) / 2.0f;
      if (centerxYellow > yThresHigh)
      {
        steerAngle = -0.5f * convertAngle;
      }
      else if (centerxYellow > yThresLow)
      {
        steerAngle = -0.45f * convertAngle;
      }
      else
      {
        steerAngle = 0.0f;
      }
    }


    if (closestYellowIndex != -1) {
      if (detectionBoundingBoxes[closestYellowIndex].y() > 0.7 && detectionBoundingBoxes[closestYellowIndex].y() < 0.5
       ) {
        steerAngle = -0.5f * convertAngle;
      }
    }

    // yBlue = detectionBoundingBoxes[boundingBoxIndexBlue].y() ;

    // float centeryBlue = (yBlue + y2Blue) / 2;

    // float centeryYellow = (yYellow + y2Yellow) / 2;

    if (verbose)
    {
      // std::cout << "xBlue " << xBlue << "x2Blue " << x2Blue << std::endl;
      // std::cout << "xYellow " << xYellow << "x2Yellow " << x2Yellow << std::endl;
      // std::cout << "Center Yellow: " << centerxYellow << std::endl;
      // std::cout << "Center Blue: " << centerxBlue << std::endl;
      // std::cout << "Center Yellow: " << centerxYellow << std::endl;
      std::cout << "Convert Angle" << convertAngle << std::endl;
      // std::cout << "CenterxYellow" << centerxYellow << std::endl;
    }
  }

  pedalPositionRequest.position(baseSpeed); // Use a set value or adaptive in nature?
  groundSteeringRequest.groundSteering(steerAngle * toRad);
  setAngle(steerAngle * toRad);

  if (verbose)
  {

    std::cout << "PP: " << pedalPositionRequest.position() << std::endl;

    std::cout << "GS rad: " << groundSteeringRequest.groundSteering() << std::endl;

    std::cout << "GS deg: " << steerAngle << std::endl;

    std::cout << "----------------------" << std::endl;
  }


  od4.send(groundSteeringRequest);
  od4.send(pedalPositionRequest);
}