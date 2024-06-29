/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <filesystem>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"
#include <boost/filesystem/operations.hpp>

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
      (0 == cmd.count("width")) || (0 == cmd.count("height")))
  {
    std::cout << argv[0]
              << " attaches to a shared memory area containing an ARGB image."
              << std::endl;
    std::cout << "Usage:   " << argv[0] << " "
              << "--cid=<OD4 session> --name=<name of shared memory area> "
                 "--width=<width of the video> --height=<height of the video> "
              << std::endl;
    std::cout
        << "Example: " << argv[0] << " "
        << "--cid=112 --name=img.argb --width=640 --height=480 "
        << std::endl;
  }
  else
  {

    std::string const name{cmd["name"]};
    std::string const property{cmd["property"]};

    uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};

    // Attach to the shared memory.
    std::unique_ptr<cluon::SharedMemory> sharedMemory{
        new cluon::SharedMemory{name}};

    while (!sharedMemory || !sharedMemory->valid())
    {
      sharedMemory.reset(new cluon::SharedMemory{name});
      if (!sharedMemory || !sharedMemory->valid())
      {
        std::clog << argv[0] << ": Failed to attach to shared memory '"
                  << name << "'. Retrying..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }

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
        cv::Mat newSrc(img.size(), CV_MAKE_TYPE(img.depth(), 4));

        int from_to[] = {0, 0, 1, 1, 2, 2, 2, 3};

        cv::mixChannels(&img, 1, &newSrc, 1, from_to, 4);

        std::string timeString = std::to_string(ts.seconds()) + std::to_string(ts.microseconds());

        std::string filename = "/images/image-" + timeString + ".png";
        cv::imshow("Image", img);
        cv::imwrite(filename, img);
        cv::waitKey(1);

        // std::cout << std::filesystem::current_path() << std::endl;

        // Convert image to BGR format.
        // cv::cvtColor(img, img, cv::COLOR_RGBA2BGR);

        // img.convertTo(img, CV_16UC3);

        // Save the image as a PNG file.

        // cv::imwrite(filename, newSrc);
        // // cv::waitKey(1);
        // std::cout << "Image saved as " << filename << std::endl;
      }
    }
    retCode = 0;
  }
  return retCode;
}
