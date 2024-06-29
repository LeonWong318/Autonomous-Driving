/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"

uint32_t const DEFAULT_MIN_EDGES = 2;
uint32_t const DEFAULT_MAX_EDGES = 10;



std::vector<std::vector<cv::Point>>  detectCones(cv::Mat& img, cv::Scalar& hsvLower,cv::Scalar& hsvUpper, uint32_t minEdges, uint32_t maxEdges,bool lookAtWholePicture = false, bool debug = false) {

    cv::Mat hsvImage, mask;
    
    cv::cvtColor(img, hsvImage, cv::COLOR_BGR2HSV);

    cv::inRange(hsvImage, hsvLower, hsvUpper, mask);

    //Only look at the road, not the upper half of the image
    if (!lookAtWholePicture) {
      int detectionRowStart = static_cast<int>(img.rows * 0.4);
      int detectionRowEnd = img.rows;

      cv::Rect bottomHalf(0, detectionRowStart, img.cols, detectionRowEnd - detectionRowStart);
      cv::Mat detectionAreaMask = cv::Mat::zeros(img.size(), CV_8UC1);
      detectionAreaMask(bottomHalf).setTo(1);

      //Find car and remove it 
      int carColSpan = static_cast<int>(img.cols * 0.25);
      int carRowStart = static_cast<int>(img.rows * 0.85);
      int carRowEnd = img.rows;
      int carColStart = img.cols / 2 - carColSpan;
      int carColEnd = img.cols / 2 + carColSpan;

      cv::Rect kiwiCar(carColStart, carRowStart, carColEnd - carColStart, carRowEnd - carRowStart);
      detectionAreaMask(kiwiCar).setTo(0);
      cv::bitwise_and(mask, detectionAreaMask, mask);

      // cv::dilate()

      if (debug) {
        cv::rectangle(img, bottomHalf, cv::Scalar(0,0,255),1);
        cv::rectangle(img, kiwiCar, cv::Scalar(255,0,255),1);
      }
    }

    cv::Mat dilate, erode, erodeImg,dilateImg;
    uint32_t iterations = 3;

    cv::dilate(mask, dilate, cv::Mat(), cv::Point(-1,1), iterations, 1,1);

    cv::erode(dilate, erode, cv::Mat(), cv::Point(-1,1), iterations, 1,1);

    std::vector<std::vector<cv::Point>> contours, filteredContours;
    cv::findContours(erode, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);


    for (const auto& contour : contours) {
      std::vector<cv::Point> approxContour, hull;
      cv::approxPolyDP(contour, approxContour, 10,true);
      cv::convexHull(approxContour, hull);

      if (hull.size() >= minEdges && hull.size() <= maxEdges){
        filteredContours.push_back(hull);
        }
    }

    return filteredContours;
}

cv::Mat drawContourBounds(cv::Mat& img, std::vector<std::vector<cv::Point>> contours, cv::Scalar color, int thickness = 1) {
  cv::drawContours(img, contours, -1, color, thickness);
  return img;
}





int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
      (0 == cmd.count("width")) || (0 == cmd.count("height"))) {
    std::cout << argv[0]
              << " attaches to a shared memory area containing an ARGB image."
              << std::endl;
    std::cout << "Usage:   " << argv[0] << " "
              << "--cid=<OD4 session> --name=<name of shared memory area> "
                 "--width=<width of the video> --height=<height of the video> "
                 "[--minArea=10] [--maxArea=10] [--verbose]"
              << std::endl;
    std::cout << "Example: " << argv[0] << " "
              << "--cid=112 --name=img.argb --width=640 --height=480 --verbose"
              << std::endl;
  } else {
    std::string const name{cmd["name"]};
    uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};


    uint32_t const minEdges = cmd.count("minArea") ? static_cast<uint32_t>(std::stoi(cmd["minEdges"])) : DEFAULT_MIN_EDGES;
    uint32_t const maxEdges = cmd.count("maxArea") ? static_cast<uint32_t>(std::stoi(cmd["maxEdges"])) : DEFAULT_MAX_EDGES;

    bool const verbose{cmd.count("verbose") != 0};
    bool const debug{cmd.count("debug") != 0};
    // Attach to the shared memory.
    std::unique_ptr<cluon::SharedMemory> sharedMemory{
        new cluon::SharedMemory{name}};
    if (sharedMemory && sharedMemory->valid()) {
      std::clog << argv[0] << ": Attached to shared memory '"
                << sharedMemory->name() << " (" << sharedMemory->size()
                << " bytes)." << std::endl;

      // Interface to a running OD4 session; here, you can send and
      // receive messages.
      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};

      // Handler to receive distance readings (realized as C++ lambda).
      std::mutex distancesMutex;
      float front{0};
      float rear{0};
      float left{0};
      float right{0};
      auto onDistance = [&distancesMutex, &front, &rear, &left, &right](
                            cluon::data::Envelope &&env) {
        auto senderStamp = env.senderStamp();
        // Now, we unpack the cluon::data::Envelope to get the desired
        // DistanceReading.
        opendlv::proxy::DistanceReading dr =
            cluon::extractMessage<opendlv::proxy::DistanceReading>(
                std::move(env));

        // Store distance readings.
        std::lock_guard<std::mutex> lck(distancesMutex);
        switch (senderStamp) {
        case 0:
          front = dr.distance();
          break;
        case 2:
          rear = dr.distance();
          break;
        case 1:
          left = dr.distance();
          break;
        case 3:
          right = dr.distance();
          break;
        }
      };
      // Finally, we register our lambda for the message identifier for
      // opendlv::proxy::DistanceReading.
      od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning()) {
        cv::Mat img;

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {

          // Copy image into cvMat structure.
          // Be aware of that any code between lock/unlock is blocking
          // the camera to provide the next frame. Thus, any
          // computationally heavy algorithms should be placed outside
          // lock/unlock
          cv::Mat wrapped(height, width, CV_8UC3, sharedMemory->data());
          img = wrapped.clone();
        }

        sharedMemory->unlock();
        std::chrono::steady_clock::time_point startTime =
            std::chrono::steady_clock::now();
        //Ola        
        //110, 50, 50
        //130, 255, 255
        cv::Scalar lowBlue(110, 50, 50);
        cv::Scalar highBlue(130, 255, 255);

        cv::Scalar lowYellow(15, 50, 50); 
        cv::Scalar highYellow(30, 255, 255);
        std::vector<std::vector<cv::Point>> blueCones, yellowCones;

        blueCones = detectCones(img, lowBlue, highBlue,  minEdges, maxEdges,false, debug);
        yellowCones = detectCones(img, lowYellow, highYellow, minEdges, maxEdges,false,  debug);

        std::chrono::steady_clock::time_point endTime =
            std::chrono::steady_clock::now();
        // Display image.
        if (verbose) {
          // int numPixels = cv::countNonZero(blueMask);
          // std::cout << "Mask has " << numPixels << " pixels." << std::endl;
          if (debug) {
            
            cv::Mat hsvBlue, hsvYellow, hsv, bottom, top,hsvBlue3, hsvYellow3;
            cv::Mat res = cv::Mat(img.rows * 2 , img.cols*2, CV_8UC3);
            cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, lowBlue, highBlue, hsvBlue);
            cv::inRange(hsv, lowYellow, highYellow, hsvYellow);


            drawContourBounds(img, blueCones, cv::Scalar(255,0,0), 3);
            drawContourBounds(img, yellowCones, cv::Scalar(0,255,255), 3);

            drawContourBounds(hsv, blueCones, cv::Scalar(255,0,0), 3);
            drawContourBounds(hsv, yellowCones, cv::Scalar(0,255,255), 3);

            cv::cvtColor(hsvBlue, hsvBlue3, cv::COLOR_GRAY2BGR);
            cv::cvtColor(hsvYellow, hsvYellow3, cv::COLOR_GRAY2BGR);

            drawContourBounds(hsvBlue3, blueCones, cv::Scalar(0,0,255), 3);
            drawContourBounds(hsvYellow3, yellowCones, cv::Scalar(0,0,255), 3);
            // top.resize(960,540);
            cv::Rect roi1(0, 0, img.cols, img.rows);
            cv::Rect roi2(img.cols, 0, img.cols, img.rows);
            cv::Rect roi3(0, img.rows, img.cols, img.rows);
            cv::Rect roi4(img.cols, img.rows, img.cols, img.rows);

          

            // Copy each image into its quadrant
            img.copyTo(res(roi1));
            hsv.copyTo(res(roi2));
            hsvYellow3.copyTo(res(roi3));
            hsvBlue3.copyTo(res(roi4));

            cv::rectangle(res, roi3, cv::Scalar(0, 255, 255), 1);
            cv::rectangle(res, roi4, cv::Scalar(255, 0, 0), 1);
            

            cv::namedWindow("debug", cv::WINDOW_NORMAL);
            cv::resizeWindow("debug", 1920, 1080);
            cv::imshow("debug", res);
            cv::imshow(sharedMemory->name().c_str(), img);
            cv::waitKey(1);

             
            std::chrono::duration<double> timeSpan =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    endTime - startTime);
            std::cout << "Processing time: " << timeSpan.count() << " seconds."
                      << std::endl;
            
          } else {

          drawContourBounds(img, blueCones, cv::Scalar(255,0,0));
          drawContourBounds(img, yellowCones, cv::Scalar(0,255,255));
          cv::imshow(sharedMemory->name().c_str(), img);
          cv::waitKey(1);
          }
        }

        ////////////////////////////////////////////////////////////////
        // Do something with the distance readings if wanted.
        {
          std::lock_guard<std::mutex> lck(distancesMutex);
          std::cout << "front = " << front << ", "
                    << "rear = " << rear << ", "
                    << "left = " << left << ", "
                    << "right = " << right << "." << std::endl;
        }


        if (blueCones.size() > 0) {
          //We have one or more cones, send message
          std::string property = "color:blue -> " + std::to_string(blueCones.size());
          opendlv::logic::perception::DetectionProperty pdp;
          pdp.property(property);
          od4.send(pdp);
        }
        if (yellowCones.size() > 0) {
          //We have one or more cones, send message
          std::string property = "color:yellow -> " + std::to_string(yellowCones.size());
          opendlv::logic::perception::DetectionProperty pdp;
          pdp.property(property);
          od4.send(pdp);
        }
        
        
      }

    }
    retCode = 0;
  }
  return retCode;
}


