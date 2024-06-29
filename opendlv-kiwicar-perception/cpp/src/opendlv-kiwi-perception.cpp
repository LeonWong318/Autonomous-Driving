/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"
#include "custom-messages.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;

vector<String> getOutputsNames(const Net &net)
{
  static vector<String> names;
  if (names.empty())
  {
    // Get the indices of the output layers, i.e. the layers with unconnected outputs
    vector<int> outLayers = net.getUnconnectedOutLayers();

    // get the names of all the layers in the network
    vector<String> layersNames = net.getLayerNames();

    // Get the names of the output layers in names
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i)
      names[i] = layersNames[outLayers[i] - 1];
  }
  return names;
}

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat &frame, vector<string> classes)
{

  Scalar paintColor;
  switch (classId)
  {
  case 0:
    paintColor = Scalar(255, 255, 255); // kiwicar (bgr --> white)
    break;
  case 1:
    paintColor = Scalar(0, 255, 255); // cone-yellow (bgr --> yellow)
    break;
  case 2:
    paintColor = Scalar(255, 130, 22); // cone-blue (bgr --> navy-blue)
    break;
  case 3:
    paintColor = Scalar(153, 255, 51); // blue-paper (bgr --> dark blue)
    break;
  case 5:
    paintColor = Scalar(153, 255, 51); // green-postit (bgr --> light green)
    break;
  // case 7:
  //   paintColor = Scalar(255, 255, 255); //truck (bgr --> white)
  //   break;
  default:
    // throw std::runtime_error("No instance of the type chosen exist, please chose right type");
    std::cout << "Found class:" << classId << std::endl;
    break;
  }
  // Draw a rectangle displaying the bounding box
  rectangle(frame, Point(left, top), Point(right, bottom), paintColor, 3);

  // Get the label for the class name and its confidence
  string label = cv::format("%.2f", conf);
  // if (!classes.empty())
  // {
  CV_Assert(classId < (int)classes.size());
  label = classes[classId] + ":" + label;
  // }

  // Display the label at the top of the bounding box
  int baseLine;
  Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
  top = max(top, labelSize.height);
  rectangle(frame, Point(left, top - static_cast<int>(round(1.5 * labelSize.height))), Point(static_cast<int>(left + round(1.5 * labelSize.width)), static_cast<int>(top + baseLine)), Scalar(255, 255, 255), FILLED);
  putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}

void postprocessNet(Mat &frame, const vector<Mat> &outs, float confThreshold, float nmsThreshold, vector<int> &classIds, vector<float> &confidences, vector<Rect> &boxes)
{
  vector<int> foundClassIds;
  vector<float> foundConfidences;
  vector<Rect> foundBoxes;
  for (size_t i = 0; i < outs.size(); ++i)

  {
    // Scan through all the bounding boxes output from the network and keep only the
    // ones with high confidence scores. Assign the box's class label as the class
    // with the highest score for the box.
    float *data = (float *)outs[i].data;
    for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
    {
      Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
      Point classIdPoint;
      double confidence;
      // Get the value and location of the maximum score
      minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
      if (confidence > confThreshold)
      {
        int centerX = (int)(data[0] * frame.cols);
        int centerY = (int)(data[1] * frame.rows);
        int width = (int)(data[2] * frame.cols);
        int height = (int)(data[3] * frame.rows);
        int left = centerX - width / 2;
        int top = centerY - height / 2;
        // cout << classIdPoint.x << endl;
        foundClassIds.push_back(classIdPoint.x);
        foundConfidences.push_back((float)confidence);
        foundBoxes.push_back(Rect(left, top, width, height));
      }
    }
  }

  // Perform non maximum suppression to eliminate redundant overlapping boxes with
  // lower confidences
  vector<int> indices;
  NMSBoxes(foundBoxes, foundConfidences, confThreshold, nmsThreshold, indices);
  for (size_t i = 0; i < indices.size(); ++i)
  {
    int idx = indices[i];
    // Rect box = foundBoxes[idx];
    boxes.push_back(foundBoxes[idx]);
    classIds.push_back(foundClassIds[idx]);
    confidences.push_back(foundConfidences[idx]);
  }
}

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
      (0 == cmd.count("width")) || (0 == cmd.count("height")) ||
      (0 == cmd.count("property")))
  {
    std::cout << argv[0]
              << " attaches to a shared taining an ARGB image."
              << std::endl;
    std::cout << "Usage:   " << argv[0] << " "
              << "--cid=<OD4 session> --name=<name of shared memory area> "
                 "--width=<width of the video> --height=<height of the video> "
                 "--classes=<classes file> "
                 "--confThreshold=<confidence threshold> "
                 "--nmsThreshold=<nms threshold>"
                 "--property=<property> [--verbose] "
                 "[--debug]"
              << std::endl;
  }
  else
  {

    string const name{cmd["name"]};
    string const property{cmd["property"]};

    uint32_t const width{static_cast<uint32_t>(stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(stoi(cmd["height"]))};

    float const confThreshold{static_cast<float>(stoi(cmd["confThreshold"]))};
    float const nmsThreshold{static_cast<float>(stoi(cmd["nmsThreshold"]))};

    string const classesFile{cmd["classes"]};

    string const cfgFile{cmd["cfgFile"]};
    string const weightsFile{cmd["weightsFile"]};

    bool const verbose{cmd.count("verbose") != 0};
    bool const debug{cmd.count("debug") != 0};

    // Load names of classes
    cout << "Loading classes" << endl;
    vector<string> classes;
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line))
    {

      cout << "Found class: " << line << endl;
      classes.push_back(line);
    }

    Scalar paintColor(0, 0, 255);

    cout << "Loading DNN model from cfg file: " << cfgFile << endl;
    // Net net = readNetFromONNX(modelPath);

    Net net = readNetFromDarknet(cfgFile, weightsFile);

    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

    if (net.empty())
    {
      cerr << "Can't load network by using the following files: " << cfgFile << endl;
      exit(-1);
    }
    else
    {
      cout << "Model loaded successfully!" << endl;
    }

    // Attach to the shared memory.
    unique_ptr<cluon::SharedMemory> sharedMemory{
        new cluon::SharedMemory{name}};

    while (!sharedMemory || !sharedMemory->valid())
    {
      sharedMemory.reset(new cluon::SharedMemory{name});
      if (!sharedMemory || !sharedMemory->valid())
      {
        clog << argv[0] << ": Failed to attach to shared memory'"
             << name << "'. Retrying..." << endl;
        this_thread::sleep_for(chrono::seconds(1));
      }
    }

    clog << argv[0] << ": Attached to shared memory '"
         << sharedMemory->name() << " (" << sharedMemory->size()
         << " bytes)." << endl;

    // Interface to a running OD4 session; here, you can send and
    // receive messages.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};

    // Endless loop; end the program by pressing Ctrl-C.
    while (od4.isRunning())
    {
      Mat img, blob;
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
        Mat wrapped(height, width, CV_8UC3, sharedMemory->data());
        img = wrapped.clone();
      }

      cvtColor(img, img, COLOR_RGBA2RGB);

      sharedMemory->unlock();

      chrono::steady_clock::time_point startTime, endTime;
      if (debug)
      {
        startTime = chrono::steady_clock::now();
      }

      // create blob to feed (ToDo figure inputs to this  (1/255 ??))
      // Input, Output, Scaling ratio of each channel, Resize, SwapRB, Crop
      blobFromImage(img, blob, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);

      net.setInput(blob);

      vector<Mat> detected;
      net.forward(detected, getOutputsNames(net));

      vector<int> classIds;
      vector<float> confidences;
      vector<Rect> boxes;

      postprocessNet(img, detected, confThreshold, nmsThreshold, classIds, confidences, boxes);

      opendlv::custom::StartFrameMessage startFrameMessage;
      startFrameMessage.numMessages(boxes.size());
      od4.send(startFrameMessage, ts);
      for (size_t i = 0; i < boxes.size(); ++i)
      {
        Rect box = boxes[i];

        opendlv::logic::perception::DetectionBoundingBox detectionBoundingBox;
        opendlv::logic::perception::DetectionType detectionType;
        opendlv::logic::perception::DetectionProperty detectionProperty;

        detectionBoundingBox.x(box.x / (float)width);
        detectionBoundingBox.y(box.y / (float)height);
        detectionBoundingBox.width(box.width / (float)width);
        detectionBoundingBox.height(box.height / (float)height);
        detectionBoundingBox.detectionId(classIds[i]);

        cout << "Detection: " << classIds[i] << " at " << box.x << ", " << box.y << " with width " << box.width << " and height " << box.height << endl;

        if (classIds[i] == 1)
        {
          // yellow/blue cone, must be set for dodo testing
          detectionProperty.property("yellow");
          detectionType.type(1);
        }
        else if (classIds[i] == 2)
        {
          detectionProperty.property("blue");
          detectionType.type(1);
        }
        else
        {
          detectionProperty.property(classes[classIds[i]]);
          detectionType.type(classIds[i]);
        }

        od4.send(detectionBoundingBox, ts);
        od4.send(detectionType, ts);
        od4.send(detectionProperty, ts);

        if (debug)
        {
          drawPred(classIds[i], confidences[i], box.x, box.y,
                   box.x + box.width, box.y + box.height, img, classes);
        }
      }
      opendlv::custom::EndFrameMessage endFrameMessage;
      od4.send(endFrameMessage, ts);

      if (debug)
      {
        endTime = chrono::steady_clock::now();
      }

      if (verbose)
      {
        imshow("Output from YOLO", img);
        waitKey(1);
      }
    }

    retCode = 0;
  }
  return retCode;
}