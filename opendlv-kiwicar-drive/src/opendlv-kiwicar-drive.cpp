/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <math.h>

#include "behavior.hpp"
#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"
#include "custom-messages.hpp"

int32_t main(int32_t argc, char **argv)
{
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if (!cmd.contains("cid"))
  {
    std::cout << argv[0] << " is an OpenDLV microservice." << std::endl;
    std::cout << "Usage: " << argv[0] << " "
              << "--cid=<conference id; e.g. 111> "
              << "--property=<property; e.g. blue or kiwicar>"
              << "--speed=<speed; e.g. 0.5>"
              << "--freq=<frequency; e.g. 10>"
              << "[--verbose] "
              << "[--debug] " << std::endl;
    return 0;
  }

  uint16_t const cid = std::stoi(cmd.at("cid"));
  std::string const property =
    cmd.contains("property") ? cmd.at("property") : "";
  bool const verbose = (cmd.count("verbose") != 0);
  bool const debug = (cmd.count("debug") != 0);
  uint16_t const freq = std::stoi(cmd.at("freq"));

  float const baseSpeed = std::stof(cmd.at("baseSpeed"));
  float const fastSpeed = std::stof(cmd.at("fastSpeed"));

  float const baseAngle = std::stof(cmd.at("baseAngle"));
  float const closeDist = std::stof(cmd.at("closeDist"));
  
  float const farDist = std::stof(cmd.at("farDist"));
  float const convertAngle = std::stof(cmd.at("convertAngle"));
  float const calibrationError = std::stof(cmd.at("calibrationError"));

  // float const DT = 1.0f / freq;
  int elapsedTime = 0;

  if (verbose)
  {
    std::cout << "Starting microservice." << std::endl;
    std::cout << "Base speed: " << baseSpeed << std::endl;
    std::cout << "Fast speed: " << fastSpeed << std::endl;
    std::cout << "Base angle: " << baseAngle << std::endl;
    std::cout << "Close distance: " << closeDist << std::endl;
    std::cout << "Far distance: " << farDist << std::endl;
    std::cout << "Convert angle: " << convertAngle << std::endl;
  }

  if (debug)
  {
    std::cout << "Debug mode is on." << std::endl;
  }

  cluon::OD4Session od4(cid);
  Behavior behavior;

  auto onDistanceReading{[&behavior](cluon::data::Envelope &&envelope) {
    auto distanceReading =
      cluon::extractMessage<opendlv::proxy::DistanceReading>(
        std::move(envelope));
    uint32_t const senderStamp = envelope.senderStamp();
    if (senderStamp == 0)
    {
      behavior.setFrontUltrasonic(distanceReading);
    }
    else if (senderStamp == 1)
    {
      behavior.setRearUltrasonic(distanceReading);
    }
    else if (senderStamp == 2)
    {
      behavior.setLeftIr(distanceReading);
    }
    else if (senderStamp == 3)
    {
      behavior.setRightIr(distanceReading);
    }
  }};

  auto onDetectionBoundingBox{[&behavior](cluon::data::Envelope &&envelope) {
    auto boundingDetection =
      cluon::extractMessage<opendlv::logic::perception::DetectionBoundingBox>(
        std::move(envelope));
    behavior.addDetectionBoundingBox(boundingDetection);
    
  }};
  auto onStartFrame{[&behavior](cluon::data::Envelope &&envelope) {
    std::cout << envelope.senderStamp() << std::endl;
    behavior.clearDetectionBoundingBoxes();
    behavior.setRecievingCameraData(true);
    behavior.setFrameId(envelope.senderStamp());
  }};

  auto onEndFrame{[&behavior](cluon::data::Envelope &&envelope) {
    std::cout << envelope.senderStamp() << std::endl;
    behavior.setRecievingCameraData(false);
  }};

  auto atFrequency{
    [&behavior, &od4, &elapsedTime, &baseSpeed, & fastSpeed ,&baseAngle, &convertAngle, &closeDist, &farDist, &verbose, &calibrationError]() -> bool {
      behavior.step(od4,baseSpeed, fastSpeed, baseAngle, convertAngle, closeDist, farDist,verbose, elapsedTime, calibrationError);
      cluon::data::TimeStamp sampleTime = cluon::time::now();
      elapsedTime ++;
      return true;
    }};

  od4.dataTrigger(opendlv::logic::perception::DetectionBoundingBox::ID(), onDetectionBoundingBox);
  od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
  od4.dataTrigger(opendlv::custom::EndFrameMessage::ID(), onEndFrame);
  od4.dataTrigger(opendlv::custom::StartFrameMessage::ID(), onStartFrame);



  // This will block until Ctrl+C is pressed.
  od4.timeTrigger(freq, atFrequency);

  if (verbose)
  {
    std::cout << "Closing microservice." << std::endl;
  }

  return 0;
}
