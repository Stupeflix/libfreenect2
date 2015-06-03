/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <stdio.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

void save_to_rgb_file(unsigned char* data, size_t dataSize, std::string prefix, std::string name, int counter)
{
    std::ostringstream counter_s;
    counter_s << std::setfill('0') << std::setw(8) << counter;

    std::stringstream ss;
    ss << prefix << "_" << name << "_" << counter_s.str() << ".raw";

    FILE* file = fopen(ss.str().c_str(), "w");
    fwrite(data, dataSize, 1, file);
    fclose(file);
}

void save_to_depth_file(float* data, size_t dataLength, std::string prefix, std::string name, int counter)
{
    std::ostringstream counter_s;
    counter_s << std::setfill('0') << std::setw(8) << counter;

    std::stringstream ss;
    ss << prefix << "_" << name << "_" << counter_s.str() << ".raw";

    FILE* file = fopen(ss.str().c_str(), "w");
    for (size_t i = 0; i < dataLength; i++)
    {
      unsigned long v = (unsigned long)(data[i] * ((1<<24) - 1));
      unsigned char r = (v & 0x0000FF) >> 0;
      unsigned char g = (v & 0x00FF00) >> 8;
      unsigned char b = (v & 0xFF0000) >> 16;
      fwrite(&r, 1, 1, file);
      fwrite(&g, 1, 1, file);
      fwrite(&b, 1, 1, file);
    }
    fclose(file);
}

bool protonect_shutdown = false;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

int main(int argc, char *argv[])
{
  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg(argv[argI]);

    if(arg == "cpu")
    {
      if(!pipeline)
        pipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(arg == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
    {
      serial = arg;
    }
    else
    {
      std::cout << "Unknown argument: " << arg << std::endl;
    }
  }

  if(pipeline)
  {
    dev = freenect2.openDevice(serial, pipeline);
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  unsigned char* registered = NULL;

  libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
  std::cout << "IR Camera Parameters:" << std::endl;
  std::cout << ir_params << std::endl;

  libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();
  std::cout << "Color Camera Parameters:" << std::endl;
  std::cout << color_params << std::endl;

  int counter = 0;
  std::string output_dir = "/tmp/output";
  std::string prefix = output_dir + "/data";
  mkdir(output_dir.c_str(), 0755);

  while(!protonect_shutdown)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    save_to_rgb_file(rgb->data, rgb->height * rgb->width * 3, prefix, "rgb", counter);

    // cv::Mat ir_mat = cv::Mat(ir->height, ir->width, CV_32FC1, ir->data);
    // cv::Mat ir_mat2;
    // ir_mat.convertTo(ir_mat2, CV_32UC1);
    // save_to_depth_file(ir_mat2.ptr<unsigned char>(), ir->height * ir->width, prefix, "ir", counter);

    cv::Mat depth_mat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    cv::Mat normalized_depth_mat = depth_mat.clone() / 4500.0f;
    save_to_depth_file(normalized_depth_mat.ptr<float>(), depth->height * depth->width, prefix, "depth", counter);

    counter += 1;
    // cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
    // cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
    cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);

    // if (!registered) registered = new unsigned char[depth->height*depth->width*rgb->bytes_per_pixel];
    // registration->apply(rgb,depth,registered);
    // cv::imshow("registered", cv::Mat(depth->height, depth->width, CV_8UC3, registered));

    int key = cv::waitKey(1);
    protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  delete[] registered;
  delete registration;

  return 0;
}
