/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iostream>
#include <iomanip>  
#include <math.h>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"

int32_t main(int32_t argc, char **argv)
{
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if (!cmd.contains("cid") || !cmd.contains("imgWidth") || !cmd.contains("imgHeight") || !cmd.contains("speed"))
  {
    std::cout << argv[0] << " is an OpenDLV microservice." << std::endl;
    std::cout << "Usage: " << argv[0] << " "
              << "--cid=<conference id; e.g. 111> "
              << "--property=<property; e.g. blue> "
              << "--speed=<speed; e.g. 0.5>"
              << "[--verbose] "
              << "[--debug] "
              << "--imgWidth=<image width; e.g. 640> "
              << "--imgHeight=<image height; e.g. 480> " << std::endl;
    return 0;
  }

  uint16_t const cid = std::stoi(cmd.at("cid"));
  std::string const property =
    cmd.contains("property") ? cmd.at("property") : "";
  bool const verbose = (cmd.count("verbose") != 0);
  bool const debug = (cmd.count("debug") != 0);
  int const imgWidth =
    cmd.contains("imgWidth") ? std::stoi(cmd.at("imgWidth")) : 0;
  int const imgHeight =
    cmd.contains("imgHeight") ? std::stoi(cmd.at("imgHeight")) : 0;

  float const speed = cmd.contains("speed") ? std::stof(cmd.at("speed")) : 0.01f;

  if (verbose)
  {
    std::cout << "Starting microservice." << std::endl;
  }

  if (debug)
  {
    std::cout << "Debug mode is on." << std::endl;
  }

  cluon::OD4Session od4(cid);

  auto onDetectionBoundingBox{
    [&od4, &imgWidth, &imgHeight, &speed](cluon::data::Envelope &&envelope) {
      auto const box =
        cluon::extractMessage<opendlv::logic::perception::DetectionBoundingBox>(
          std::move(envelope));

      // Normalize the bounding box
      float x1 = box.x() / imgWidth;
      float y2 = box.y() / imgHeight;
      float x2 = (box.x() + box.width()) / imgWidth;
      float y1 = (box.y() + box.height()) / imgHeight;

      float centerX = (x1 + x2) / 2;
      float centerY = (y1 + y2) / 2;

      //We want to map -38 degrees to 38 degrees to 0 to 1

      // x = 0 goes to -38, x = 1 goes to 38
      // carSpeed range is +0.25 forwards, -1 backwards
      float angle = (x1 - 0.5f) * 76;
      float steerAngle, carSpeed;
      opendlv::proxy::GroundSteeringRequest gsr;
      opendlv::proxy::PedalPositionRequest ppr;

      if (centerX < 0.5)
      {
        std::cout << "Object is on the left " << std::fixed << std::setprecision(2) << centerX << "," << centerY << " Angle: " << angle << std::endl;
        //ToDo use compute to calculate steering angle (closer to centroid less steer and farther more steer)
        steerAngle = (-angle/180) * static_cast<float>(M_PI); //Added -ve sign as left turn is +ve angle
        carSpeed = speed;
      }
      else
      {
        std::cout << "Object is on the right " << std::fixed << std::setprecision(2) << centerX << "," << centerY  << " Angle: " << angle << std::endl;
        steerAngle = (- angle/180) * static_cast<float>(M_PI);  //Added -ve sign as right turn is -ve angle
        carSpeed = speed;
      }

      if (centerY > 0.8) {
        std::cout << "Object is close. Should stop" << std::endl;
        //Stop car
        carSpeed = 0;
        steerAngle = 0;
      }

      ppr.position(carSpeed); //Use a set value or adaptive in nature?
      od4.send(ppr);

      gsr.groundSteering(steerAngle);
      od4.send(gsr);

      //ADD WIGGLE FEATURE HERE

    }};

  od4.dataTrigger(opendlv::logic::perception::DetectionBoundingBox::ID(),
                  onDetectionBoundingBox);

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
