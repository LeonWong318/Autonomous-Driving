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
#include <random>

float const PI = 3.14159265358979323846f;
float const toRad = PI / 180.0f;
int const CLASS_KIWICAR = 0;
int const CLASS_CONE_BLUE = 1;
int const CLASS_CONE_YELLOW = 2;
int const CLASS_PAPER_BLUE = 3;
int const CLASS_POSTIT_GREEN = 4;

std::string mode = "searching";

Behavior::Behavior() noexcept
  : m_frontUltrasonicReading{}, m_rearUltrasonicReading{}, m_leftIrReading{},
    m_rightIrReading{}, m_detectionBoundingBox{}, m_detectionBoundingBoxes{},
    m_recievingData{}, m_frameId{}, m_lastRandomAngle{}, m_frontUltrasonicReadingMutex{},
    m_rearUltrasonicReadingMutex{}, m_leftIrReadingMutex{},
    m_rightIrReadingMutex{}, m_detectionBoundingBoxMutex{},
    m_detectionBoundingBoxesMutex(), m_recievingDataMutex{}, m_frameIdMutex(), m_lastRandomAngleMutex{}
{
  srand (static_cast <unsigned> (time(0)));
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

void Behavior::setRandomAngle(float angle) noexcept
{
  std::lock_guard<std::mutex> lock(m_lastRandomAngleMutex);
  m_lastRandomAngle = angle;
}

void Behavior::step(cluon::OD4Session &od4, float baseSpeed, float fastSpeed,
                    float baseAngle, float convertAngle, float closeDist,
                    float farDist, bool verbose, int elapsedTime, float calibrationError) noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::DistanceReading leftIrReading;
  opendlv::proxy::DistanceReading rightIrReading;
  std::vector<opendlv::logic::perception::DetectionBoundingBox>
    detectionBoundingBoxes;
  bool recievingData;
  float lastRandomAngle;
  uint32_t frameId;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);
    std::lock_guard<std::mutex> lock5(m_detectionBoundingBoxesMutex);
    std::lock_guard<std::mutex> lock6(m_recievingDataMutex);
    std::lock_guard<std::mutex> lock7(m_frameIdMutex);
    std::lock_guard<std::mutex> lock8(m_lastRandomAngleMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
    detectionBoundingBoxes = m_detectionBoundingBoxes;
    recievingData = m_recievingData;
    frameId = m_frameId;
    lastRandomAngle = m_lastRandomAngle;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = leftIrReading.distance();
  double rightDistance = rightIrReading.distance();

  opendlv::proxy::PedalPositionRequest pedalPositionRequest;
  opendlv::proxy::GroundSteeringRequest groundSteeringRequest;
  int boundingBoxIndex = -1;

  for (size_t i = 0; i < detectionBoundingBoxes.size(); i++)
  {
    if (detectionBoundingBoxes[i].detectionId() == CLASS_KIWICAR)
    {
      float area =
        detectionBoundingBoxes[i].width() * detectionBoundingBoxes[i].height();
      if (boundingBoxIndex == -1)
      {
        boundingBoxIndex = i;
      }
      else if (area > detectionBoundingBoxes[boundingBoxIndex].width() *
                        detectionBoundingBoxes[boundingBoxIndex].height())
      {
        boundingBoxIndex = i;
      }
    }
  }

  if (boundingBoxIndex != -1)
  {
    mode = "following";
  }
  else
  {
    mode = "searching";
  }

  if (verbose)
  {

    std::cout << "-----------" << mode <<"-----------" << std::endl;
    std::cout << "Front distance: " << frontDistance << std::endl;
    std::cout << "Rear distance: " << rearDistance << std::endl;
    std::cout << "Left distance: " << leftDistance << std::endl;
    std::cout << "Right distance: " << rightDistance << std::endl;
    std::cout << "Frame id: " << frameId << std::endl;
    std::cout << "Recieving data: " << recievingData << std::endl;
    std::cout << "Num detection bounding boxes: "
              << m_detectionBoundingBoxes.size() << std::endl;
    std::cout << "Convert angle: " << convertAngle << std::endl;
    std::cout << "----------------------" << std::endl;
  }




  if (mode == "searching")
  {
    if (frontDistance -calibrationError < closeDist)
    {
      // We are close to a wall, reverse
      pedalPositionRequest.position(-1);
      groundSteeringRequest.groundSteering(0.0);
    }
    else if (frontDistance - calibrationError < farDist)
    {
      // We are close to a wall, but not really
      pedalPositionRequest.position(baseSpeed);
      if (leftDistance <= rightDistance)
      {
        // Go right
        groundSteeringRequest.groundSteering(-baseAngle * toRad);
      }
      else
      {
        // Go left
        groundSteeringRequest.groundSteering(baseAngle * toRad);
      }
    }
    else
    {
      std::cout << "Elapsed time: " << elapsedTime << std::endl;
      std::cout << "Random angle: " << lastRandomAngle << std::endl;
    
      if (elapsedTime % 10 == 0)
      {
        std::cout << "Randomizing angle" << std::endl;
        float randAngle = static_cast <float> (rand()) / ((static_cast <float> (RAND_MAX)) * 76.0f);
        randAngle = randAngle - 38.0f;
        setRandomAngle(randAngle);
        groundSteeringRequest.groundSteering(randAngle * toRad);
      } else {
        std::cout << "Using last random angle" << std::endl;
        groundSteeringRequest.groundSteering(lastRandomAngle * toRad);
      }
      pedalPositionRequest.position(baseSpeed);
    }
  }

  if (mode == "following")
  {

    // We want to map -38 degrees to 38 degrees to 0 to 1
    float centerX = (detectionBoundingBoxes[boundingBoxIndex].x() +
                     detectionBoundingBoxes[boundingBoxIndex].width() / 2);
    //float centerY = (detectionBoundingBoxes[boundingBoxIndex].y() +
     //                detectionBoundingBoxes[boundingBoxIndex].height() / 2);
    float angle;
    float carSpeed;

    // Check front distance reading, if it's too close, slow down, if far,
    // speed up
    if (frontDistance - calibrationError < closeDist && frontDistance - calibrationError > 0.0f)
    {
      carSpeed = 0.0f;
    }
    else if (frontDistance - calibrationError < farDist && frontDistance - calibrationError > 0.0f)
    {
      carSpeed = baseSpeed;
    }
    else
    {
      carSpeed = fastSpeed;
    }

    // angle = std::atan2(0.5f - centerX, 1.0f - centerY);
    angle = centerX -0.5f * convertAngle;


    groundSteeringRequest.groundSteering(angle * toRad);
    pedalPositionRequest.position(carSpeed);
  }

  std::cout << "Pedal position: " << pedalPositionRequest.position()
            << std::endl;
  std::cout << "Ground steering: " << groundSteeringRequest.groundSteering()
            << std::endl;

  // Send the pedal position request
  od4.send(pedalPositionRequest);
  // Send the ground steering request
  od4.send(groundSteeringRequest);
}
