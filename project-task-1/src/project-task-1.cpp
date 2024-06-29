/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iostream>
#include <iomanip>  
#include <math.h>
#include <map>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"
#include "messages.hpp"
#include "behavior.hpp"
#include <opencv2/highgui/highgui.hpp>


int32_t main(int32_t argc, char **argv)
{ 
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if (!cmd.contains("cid"))
  {
    std::cout << argv[0] << " is an OpenDLV microservice." << std::endl;
    std::cout << "Usage: " << argv[0] << " "
              << "--cid=<conference id; e.g. 111> "
              << "[--verbose] "
              << "[--debug] "
              << "--freq=<frequency;e.g. 10>"
              << "--speed=<speed; e.g. 0.5>";
    return 0;
  }

  uint16_t const cid = std::stoi(cmd.at("cid"));
  bool const verbose = (cmd.count("verbose") != 0);
  bool const debug = (cmd.count("debug") != 0);
  uint16_t const freq = std::stoi(cmd.at("freq"));

  int baseSpeed{static_cast<int>(std::stoi(cmd.at("baseSpeed")))};
  int convertAngle{static_cast<int>(std::stoi(cmd.at("convertAngle")))};
  // int threshold{static_cast<int>(std::stoi(cmd.at("threshold")))};
  int lowYThreshold{static_cast<int>(std::stoi(cmd.at("lowYThreshold")))};
  int highYThreshold{static_cast<int>(std::stoi(cmd.at("highYThreshold")))};
  int lowXThreshold{static_cast<int>(std::stoi(cmd.at("lowXThreshold")))};
  int highXThreshold{static_cast<int>(std::stoi(cmd.at("highXThreshold")))};
  // float const DT = 1.0f / freq;
  float elapsedTime = 0.0f;
  

  if (verbose)
  {
    std::cout << "Starting microservice." << std::endl;
    std::cout << "Base speed: " << baseSpeed << std::endl;
    std::cout << "Convert angle: " << convertAngle << std::endl;
  }

  if (debug)
  {
    std::cout << "Debug mode is on." << std::endl;

    std::string controlWindow = "Velocity & Angle control";
    cv::namedWindow(controlWindow, cv::WINDOW_AUTOSIZE);

    
    cv::createTrackbar("Speed", controlWindow, &baseSpeed, 100);
    cv::createTrackbar("Convert angle", controlWindow, &convertAngle, 100);
    // cv::createTrackbar("Threshold", controlWindow, &threshold, 100);
    cv::createTrackbar("X low", controlWindow, &lowXThreshold, 100);
    cv::createTrackbar("X high", controlWindow, &highXThreshold, 100);

    cv::createTrackbar("Y low", controlWindow, &lowYThreshold, 100);
    cv::createTrackbar("Y high", controlWindow, &highYThreshold, 100);


  }

  cluon::OD4Session od4(cid);
  Behavior behavior;
  
  auto onStartFrame{[&behavior](cluon::data::Envelope &&envelope) {
     
      behavior.clearDetectionBoundingBoxes();
      behavior.setRecievingCameraData(true);
      behavior.setFrameId(envelope.senderStamp());
  }};

  auto onEndFrame{[&behavior](cluon::data::Envelope &&envelope) {
    behavior.setRecievingCameraData(false);
    behavior.setFrameId(envelope.senderStamp());
  }};

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

  od4.dataTrigger(opendlv::logic::perception::DetectionBoundingBox::ID(),
                  onDetectionBoundingBox);
  od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
  od4.dataTrigger(opendlv::custom::EndFrameMessage::ID(), onEndFrame);
  od4.dataTrigger(opendlv::custom::StartFrameMessage::ID(), onStartFrame);

  auto atFrequency{
    [&behavior, &od4, &baseSpeed, &elapsedTime, &convertAngle, &verbose, &lowXThreshold, &highXThreshold, &lowYThreshold, &highYThreshold]() -> bool {
      

      cv::waitKey(1); 
      float speed = baseSpeed / 100.0f;

      float xThresLow = lowXThreshold / 100.0f;
      float xThresHigh = highXThreshold / 100.0f;

      float yThresLow = lowYThreshold / 100.0f;
      float yThresHigh = highYThreshold / 100.0f;



      behavior.step(od4, speed, convertAngle, verbose,xThresLow,xThresHigh,yThresLow,yThresHigh );
      
      cluon::data::TimeStamp sampleTime = cluon::time::now();
      
      // elapsedTime += DT;
      return true;
    }};
    
  od4.timeTrigger(freq, atFrequency);

  while (od4.isRunning())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (verbose)
  {
    std::cout << "Closing microservice." << std::endl;
  }

  return 0;
}
