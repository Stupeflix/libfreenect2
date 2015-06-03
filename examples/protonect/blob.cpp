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

bool protonect_shutdown = false;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

float blur_depth(float* values,
                 int width,
                 int height,
                 int blur_x,
                 int blur_y,
                 int radius,
                 float max_delta)
{
    int num_samples = 0;
    float acc = 0;
    float center_depth = values[blur_y * width + blur_x];

    for (int y = blur_y - radius; y <= blur_y + radius; y++) {
        if ((y > 0) && (y < height)) {
            for (int x = blur_x - radius; x <= blur_x + radius; x++) {
                if ((x > 0) && (x < width)) {
                    float depth = values[y * width + x];
                    if (depth && (std::abs(depth - center_depth) < max_delta)) {
                        acc += depth;
                        num_samples++;
                    }
                }
            }
        }
    }

    if (num_samples > 4) {
        return acc / num_samples;
    } else {
        return 4500;
    }
}

void update_pixels(libfreenect2::Frame* rgb_frame,
                   libfreenect2::Frame* depth_frame,
                   libfreenect2::Registration* registration,
                   float* accumulated_depth_frame,
                   unsigned char* out)
{
    for (size_t y = 0; y < depth_frame->height; y++) {
        for (size_t x = 0; x < depth_frame->width; x++) {
            size_t depth_index = y * depth_frame->width + x;

            //float depth = ((float*)depth_frame->data)[depth_index];
            float depth = blur_depth((float*)depth_frame->data, depth_frame->width, depth_frame->height, x, y, 1, 50);
            float accumulated_depth = accumulated_depth_frame[depth_index];

            if (depth < accumulated_depth) {
                float rgb_x, rgb_y;
                registration->apply(x, y, depth, rgb_x, rgb_y);
                size_t rgb_offset = (round(rgb_x) + round(rgb_y) * rgb_frame->width) * rgb_frame->bytes_per_pixel;
                size_t out_offset = depth_index * 3;

                if ((rgb_offset >= 0) && (rgb_offset < rgb_frame->width * rgb_frame->height * rgb_frame->bytes_per_pixel)) {
                    out[out_offset + 0] = rgb_frame->data[rgb_offset + 0];
                    out[out_offset + 1] = rgb_frame->data[rgb_offset + 1];
                    out[out_offset + 2] = rgb_frame->data[rgb_offset + 2];
                } else {
                    out[out_offset + 0] = 0;
                    out[out_offset + 1] = 0;
                    out[out_offset + 2] = 0;
                }

                accumulated_depth_frame[depth_index] = depth;
            }
        }
    }
}

float fill_float_buffer(float* buf, size_t size, float value)
{
    for (size_t i = 0; i < size; i++) {
        buf[i] = 10000.0;
    }
}

int main(int argc, char *argv[])
{
  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("blob");

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

  std::string output_dir = "/tmp/output";
  std::string prefix = output_dir + "/data";
  mkdir(output_dir.c_str(), 0755);

  float* accumulated_depth = NULL;
  unsigned char* image = NULL;
  size_t frame = 0;
  size_t num_depth_pixels;

  while(!protonect_shutdown)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    // Initialize depth buffers and copy kinect depth to the current read
    // buffer
    if (!accumulated_depth) {
        num_depth_pixels = depth->width * depth->height;
        accumulated_depth = new float[num_depth_pixels];
        image = new unsigned char[num_depth_pixels * 3];
        memset(image, 0, num_depth_pixels * 3);
    }
    if ((frame % (60 * 5)) == 0) {
        fill_float_buffer(accumulated_depth, num_depth_pixels, 4500);
    }
    update_pixels(rgb, depth, registration, accumulated_depth, image);

    cv::imshow("blob", cv::Mat(depth->height, depth->width, CV_8UC3, image));
    //cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);

    int key = cv::waitKey(1);
    protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

    listener.release(frames);
    frame++;
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  delete[] registered;
  delete registration;

  return 0;
}
