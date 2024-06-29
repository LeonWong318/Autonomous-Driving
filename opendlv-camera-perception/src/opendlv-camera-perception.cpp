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
#include "custom-messages.hpp"
#include "opendlv-message-standard.hpp"

std::vector<std::vector<cv::Point>> detectCones(
  cv::Mat &img, cv::Scalar &hsvLower, cv::Scalar &hsvUpper, uint32_t erosions,
  uint32_t dilations, uint32_t minEdges, uint32_t maxEdges,
  bool lookAtWholePicture = false, bool debug = false, int xThreshold = 50,
  int xThresholdCar = 70)
{

  cv::Mat hsvImage, mask;

  cv::cvtColor(img, hsvImage, cv::COLOR_BGR2HSV);

  cv::inRange(hsvImage, hsvLower, hsvUpper, mask);

  // Only look at the road, not the upper half of the image
  if (!lookAtWholePicture)
  {
    int detectionRowStart = static_cast<int>(img.rows * xThreshold / 100.0f);
    int detectionRowEnd = img.rows;

    cv::Rect bottomHalf(0, detectionRowStart, img.cols,
                        detectionRowEnd - detectionRowStart);
    cv::Mat detectionAreaMask = cv::Mat::zeros(img.size(), CV_8UC1);
    detectionAreaMask(bottomHalf).setTo(1);

    // Find car and remove it
    int carRowStart = static_cast<int>(img.rows * xThresholdCar / 100.0f);
    int carRowEnd = img.rows;
    int carColStart = 0;
    int carColEnd = img.cols;

    cv::Rect kiwiCar(carColStart, carRowStart, carColEnd - carColStart,
                     carRowEnd - carRowStart);
    detectionAreaMask(kiwiCar).setTo(0);
    cv::bitwise_and(mask, detectionAreaMask, mask);

    // cv::dilate()

    if (debug)
    {
      cv::rectangle(img, bottomHalf, cv::Scalar(0, 0, 255), 1);
      cv::rectangle(img, kiwiCar, cv::Scalar(255, 0, 255), 1);
    }
  }

  cv::Mat dilate, erode, erodeImg, dilateImg;
  cv::dilate(mask, dilate, cv::Mat(), cv::Point(-1, 1), dilations, 1, 1);

  cv::erode(dilate, erode, cv::Mat(), cv::Point(-1, 1), erosions, 1, 1);

  std::vector<std::vector<cv::Point>> contours, filteredContours;
  cv::findContours(erode, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  for (const auto &contour : contours)
  {
    std::vector<cv::Point> approxContour, hull;
    cv::approxPolyDP(contour, approxContour, 10, true);
    cv::convexHull(approxContour, hull);

    if (hull.size() >= minEdges && hull.size() <= maxEdges)
    {
      filteredContours.push_back(hull);
    }
  }

  return filteredContours;
}

cv::Mat drawContourBounds(cv::Mat &img,
                          std::vector<std::vector<cv::Point>> contours,
                          cv::Scalar color, int thickness = 1)
{
  cv::drawContours(img, contours, -1, color, thickness);
  return img;
}

int detectionId = 0;

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
      (0 == cmd.count("width")) || (0 == cmd.count("height")) ||
      (0 == cmd.count("property")) || (0 == cmd.count("erosions")) ||
      (0 == cmd.count("dilations")) || (0 == cmd.count("hueLow")) ||
      (0 == cmd.count("hueHigh")) || (0 == cmd.count("satLow")) ||
      (0 == cmd.count("satHigh")) || (0 == cmd.count("valLow")) ||
      (0 == cmd.count("valHigh")) || (0 == cmd.count("minEdges")) ||
      (0 == cmd.count("maxEdges")))
  {
    std::cout << argv[0]
              << " attaches to a shared memory area containing an ARGB image."
              << std::endl;
    std::cout << "Usage:   " << argv[0] << " "
              << "--cid=<OD4 session> --name=<name of shared memory area> "
                 "--width=<width of the video> --height=<height of the video> "
                 "--property=<property> --erosions=<number of erosions> "
                 "--dilations=<number of dilations> --hueLow=<hue low value> "
                 "--hueHigh=<hue high value> --satLow=<saturation low value> "
                 "--satHigh=<saturation high value> --valLow=<value low value> "
                 "--valHigh=<value high value> --minEdges=<minimum edges> "
                 "--maxEdges=<maximum edges> --lookAtWholePicture [--verbose] "
                 "[--debug] [--controlHsv]"
              << std::endl;
    std::cout
      << "Example: " << argv[0] << " "
      << "--cid=112 --name=img.argb --width=640 --height=480 "
         "--property=propertyValue "
         "--erosions=2 --dilations=2 --hueLow=0 --hueHigh=255 --satLow=0 "
         "--satHigh=255 --valLow=0 --valHigh=255 --minEdges=10 --maxEdges=100 "
         "--lookAtWholePicture --verbose --debug --controlHsv"
      << std::endl;
  }
  else
  {

    std::string const name{cmd["name"]};
    std::string const property{cmd["property"]};

    int erosions{static_cast<int>(std::stoi(cmd["erosions"]))};
    int dilations{static_cast<int>(std::stoi(cmd["dilations"]))};

    int hueLow{static_cast<int>(std::stoi(cmd["hueLow"]))};
    int hueHigh{static_cast<int>(std::stoi(cmd["hueHigh"]))};
    int satLow{static_cast<int>(std::stoi(cmd["satLow"]))};
    int satHigh{static_cast<int>(std::stoi(cmd["satHigh"]))};
    int valLow{static_cast<int>(std::stoi(cmd["valLow"]))};
    int valHigh{static_cast<int>(std::stoi(cmd["valHigh"]))};
    int xThreshold{static_cast<int>(std::stoi(cmd["xThreshold"]))};
    int xThresholdCar{static_cast<int>(std::stoi(cmd["xThresholdCar"]))};
    // int yThresholdCar{static_cast<int>(std::stoi(cmd["yThresholdCar"]))};

    uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};

    int minEdges{static_cast<int>(std::stoi(cmd["minEdges"]))};
    int maxEdges{static_cast<int>(std::stoi(cmd["maxEdges"]))};
    bool const lookAtWholePicture{cmd.count("lookAtWholePicture") != 0};

    bool const verbose{cmd.count("verbose") != 0};
    bool const debug{cmd.count("debug") != 0};
    bool const controlHsv{cmd.count("controlHsv") != 0};

    if (controlHsv)
    {

      std::string controlWindow = "Control" + property;

      cv::namedWindow(controlWindow, cv::WINDOW_NORMAL);

      if (property == "blue")
      {
        cv::moveWindow(controlWindow, 600, 20);
      }
      else
      {
        cv::moveWindow(controlWindow, 800, 20);
      }

      cv::createTrackbar("LowH", controlWindow, &hueLow, 179); // Hue (0 - 179)
      cv::createTrackbar("LowS", controlWindow, &satLow,
                         255); // Saturation (0 - 255)
      cv::createTrackbar("LowV", controlWindow, &valLow,
                         255); // Value (0 - 255)

      cv::createTrackbar("HighH", controlWindow, &hueHigh, 179);
      cv::createTrackbar("HighS", controlWindow, &satHigh, 255);
      cv::createTrackbar("HighV", controlWindow, &valHigh, 255);
      cv::createTrackbar("Erosions", controlWindow, &erosions, 10);
      cv::createTrackbar("Dilations", controlWindow, &dilations, 10);
      cv::createTrackbar("MinEdges", controlWindow, &minEdges, 10);
      cv::createTrackbar("MaxEdges", controlWindow, &maxEdges, 10);
      cv::createTrackbar("X-threshold", controlWindow, &xThreshold, 100);
      cv::createTrackbar("X-threshold-car", controlWindow, &xThresholdCar, 100);
    }

    cv::Scalar paintColor(0, 0, 255);

    // Attach to the shared memory.
    std::unique_ptr<cluon::SharedMemory> sharedMemory{
      new cluon::SharedMemory{name}};

    if (sharedMemory && sharedMemory->valid())
    {
      std::clog << argv[0] << ": Attached to shared memory '"
                << sharedMemory->name() << " (" << sharedMemory->size()
                << " bytes)." << std::endl;

      // Interface to a running OD4 session; here, you can send and
      // receive messages.
      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning())
      {
        cv::Mat img;
        cluon::data::TimeStamp ts;

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
          ts = sharedMemory->getTimeStamp().second;
          cv::Mat wrapped(height, width, CV_8UC3, sharedMemory->data());
          img = wrapped.clone();
        }

        sharedMemory->unlock();

        std::chrono::steady_clock::time_point startTime, endTime;
        if (debug)
        {
          startTime = std::chrono::steady_clock::now();
        }
        // Ola
        // 110, 50, 50
        // 130, 255, 255
        cv::Scalar lowThreshold(hueLow, satLow, valLow);
        cv::Scalar highThreshold(hueHigh, satHigh, valHigh);
        std::vector<std::vector<cv::Point>> cones;

        cones = detectCones(img, lowThreshold, highThreshold, erosions,
                            dilations, minEdges, maxEdges, lookAtWholePicture,
                            debug, xThreshold, xThresholdCar);
        // Send start of frame
        opendlv::custom::StartFrameMessage startFrameMessage;
        startFrameMessage.numMessages(cones.size());
        od4.send(startFrameMessage, ts);

        // Bounding boxes
        for (const auto &cone : cones)
        {
          cv::Rect boundingBox = cv::boundingRect(cone);
          if (verbose)
          {
            cv::rectangle(img, boundingBox, paintColor, 1);
          }

          float normalizedX =
            static_cast<float>(boundingBox.x) / static_cast<float>(width);
          float normalizedY =
            static_cast<float>(boundingBox.y) / static_cast<float>(height);
          float normalizedWidth =
            static_cast<float>(boundingBox.width) / static_cast<float>(width);
          float normalizedHeight =
            static_cast<float>(boundingBox.height) / static_cast<float>(height);

          if (debug)
          {

            std::cout << "Cone id " << detectionId
                      << " detected at: " << normalizedX << ", " << normalizedY
                      << " with width: " << normalizedWidth
                      << " and height: " << normalizedHeight << std::endl;
          }

          // We have one or more cones, send message
          std::string prop = "color:" + property;
          opendlv::logic::perception::DetectionProperty detectionProperty;
          detectionProperty.property(prop);
          detectionProperty.detectionId(detectionId);

          opendlv::logic::perception::DetectionType detectionType;

          detectionType.type(1);
          detectionType.detectionId(detectionId);

          opendlv::logic::perception::DetectionBoundingBox detectionBoundingBox;
          detectionBoundingBox.x(normalizedX);
          detectionBoundingBox.y(normalizedY);
          detectionBoundingBox.width(normalizedWidth);
          detectionBoundingBox.height(normalizedHeight);
          detectionBoundingBox.detectionId((property == "blue") ? 1 : 2);

          od4.send(detectionProperty, ts);
          od4.send(detectionType, ts);
          od4.send(detectionBoundingBox, ts);
          detectionId++;
        }
        opendlv::custom::EndFrameMessage endFrameMessage;
        od4.send(endFrameMessage, ts);

        if (debug)
        {
          endTime = std::chrono::steady_clock::now();
        }

        // Display image.
        if (verbose)
        {
          // int numPixels = cv::countNonZero(blueMask);
          // std::cout << "Mask has " << numPixels << " pixels." << std::endl;
          if (debug)
          {

            drawContourBounds(img, cones, cv::Scalar(0, 255, 0), 1);
            cv::imshow(property, img);
            cv::waitKey(1);

            std::chrono::duration<double> timeSpan =
              std::chrono::duration_cast<std::chrono::duration<double>>(
                endTime - startTime);
            std::cout << "Processing time: " << timeSpan.count() << " seconds."
                      << std::endl;
          }
          else
          {
            cv::imshow(property, img);
            cv::waitKey(1);
          }
        }
        if (controlHsv)
        {
          // 2. Create a new HSV image with the same size as the original
          // image.
          cv::Mat hsvImage = cv::Mat::zeros(img.size(), CV_8UC3);

          // 3. Fill the new HSV image with a gradient from hsvLow to hsvHigh.
          for (int i = 0; i < hsvImage.rows; i++)
          {
            for (int j = 0; j < hsvImage.cols; j++)
            {
              for (int k = 0; k < 3; k++)
              {
                auto res = (double)(highThreshold[k] - lowThreshold[k]);
                auto res2 = (double)j / hsvImage.cols;

                hsvImage.at<cv::Vec3b>(i, j)[k] =
                  static_cast<unsigned char>(lowThreshold[k] + res * res2);
              }
            }
          }

          // 4. Convert the HSV image to BGR color space for display.
          cv::Mat bgrImage;
          cv::cvtColor(hsvImage, bgrImage, cv::COLOR_HSV2BGR);
          cv::imshow("Color", bgrImage);
        }
      }
    }
    retCode = 0;
  }
  return retCode;
}
