/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <openni_camera/openni_driver.h>
#include <openni_camera/openni_device.h>
#include <openni_camera/openni_image.h>
#include <openni_camera/openni_depth_image.h>
#include <iostream>
#include <string>
#include <map>
#include <XnCppWrapper.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <sys/times.h>

using namespace std;
using namespace openni_wrapper;
using namespace cv;
using namespace boost;

class MyOpenNIExample
{
public:

  typedef struct ImgContext : public boost::noncopyable
  {

    ImgContext () : is_new (false)
    {
    }

    ImgContext (const Mat & img) : image (img), is_new (false)
    {
    }
    Mat image;
    mutable boost::mutex lock;
    bool is_new;
  } ImageContext;
public:
  MyOpenNIExample (const vector<unsigned>& device_indices);
  ~MyOpenNIExample ();
  int run ();
private:
  void imageCallback (boost::shared_ptr<Image> image, void* cookie);
  void depthCallback (boost::shared_ptr<DepthImage> depth, void* cookie);
  void writeImages () const;
  map<string, ImageContext*> rgb_images_;
  map<string, ImageContext*> gray_images_;
  map<string, ImageContext*> depth_images_;
  vector< boost::shared_ptr<OpenNIDevice> > devices_;
  bool running_;
  unsigned selected_device_;

  double image_timestamp;
  double depth_timestamp;
};

MyOpenNIExample::MyOpenNIExample (const vector<unsigned>& device_indices)
: running_ (false)
, selected_device_ (0)
{
  OpenNIDriver& driver = OpenNIDriver::getInstance ();

  for (vector<unsigned>::const_iterator indexIt = device_indices.begin (); indexIt != device_indices.end (); ++indexIt)
  {
    if (*indexIt >= driver.getNumberDevices ())
    {
      cout << "Index out of range." << driver.getNumberDevices () << " devices found." << endl;
      exit (1);
    }

    boost::shared_ptr<OpenNIDevice> device = driver.getDeviceByIndex (*indexIt);
    cout << devices_.size () + 1 << ". device on bus: " << (int)device->getBus () << " @ " << (int)device->getAddress ()
            << " with serial number: " << device->getSerialNumber () << "  "
            << device->getVendorName () << " : " << device->getProductName () << endl;
    devices_.push_back (device);

    const int width = 640;
    const int height = 480;
    XnMapOutputMode mode;
    mode.nXRes = width;
    mode.nYRes = height;
    mode.nFPS = 30;
    
    if (device->hasImageStream())
    {
      if (!device->isImageModeSupported (mode))
      {
        cout << "image stream mode " << mode.nXRes << " x " << mode.nYRes << " @ " << mode.nFPS << " not supported" << endl;
        exit (-1);
      }
      namedWindow (string (device->getConnectionString ()) + "RGB", WINDOW_AUTOSIZE);
      namedWindow (string (device->getConnectionString ()) + "Gray", WINDOW_AUTOSIZE);
      rgb_images_[device->getConnectionString ()] = new ImageContext (Mat::zeros (height, width, CV_8UC3));
      gray_images_[device->getConnectionString ()] = new ImageContext (Mat::zeros (height, width, CV_8UC1));
      device->registerImageCallback (&MyOpenNIExample::imageCallback, *this, &(*device));
    }
    if (device->hasDepthStream())
    {
      if (!device->isDepthModeSupported (mode))
      {
        cout << "depth stream mode " << mode.nXRes << " x " << mode.nYRes << " @ " << mode.nFPS << " not supported" << endl;
        exit (-1);
      }
      namedWindow (string (device->getConnectionString ()) + "Depth", WINDOW_AUTOSIZE);
      depth_images_[device->getConnectionString ()] = new ImageContext (Mat::zeros (height, width, CV_32FC1));
      device->registerDepthCallback (&MyOpenNIExample::depthCallback, *this, &(*device));
    }
  }

  timeval timestamp;
  gettimeofday (&timestamp, NULL);
  image_timestamp = depth_timestamp = timestamp.tv_sec + timestamp.tv_usec * 0.000001;
}

MyOpenNIExample::~MyOpenNIExample ()
{
  // this should call the device destructors, which are blocking until workerthreads return.
  devices_.clear ();

  //now its save to free images
  for (map<string, ImageContext*>::iterator imageIt = rgb_images_.begin (); imageIt != rgb_images_.end (); ++imageIt)
  {
    delete imageIt->second;
  }

  for (map<string, ImageContext*>::iterator imageIt = gray_images_.begin (); imageIt != gray_images_.end (); ++imageIt)
  {
    delete imageIt->second;
  }

  for (map<string, ImageContext*>::iterator imageIt = depth_images_.begin (); imageIt != depth_images_.end (); ++imageIt)
  {
    delete imageIt->second;
  }
}

void MyOpenNIExample::writeImages () const
{
  cout << "write images" << endl;
  static unsigned index = 0;
  ++index;

  cout << "locking rgb images..." << flush;
  map<string, ImageContext*>::const_iterator imageIt;
  for (imageIt = rgb_images_.begin (); imageIt != rgb_images_.end (); ++imageIt)
    imageIt->second->lock.lock ();

  cout << "done\nlocking gray images..." << flush;
  for (imageIt = gray_images_.begin (); imageIt != gray_images_.end (); ++imageIt)
    imageIt->second->lock.lock ();
  //  for (imageIt = depth_images_.begin (); imageIt != depth_images_.end (); ++imageIt)
  //    imageIt->second->lock.lock ();
  
  cout << "locking rgb images..." << flush;
  char file_name[255];
  for (imageIt = rgb_images_.begin (); imageIt != rgb_images_.end (); ++imageIt)
  {
    sprintf (file_name, "rgb_%03u.png", index);
    imwrite (file_name, imageIt->second->image);
  }

  for (imageIt = gray_images_.begin (); imageIt != gray_images_.end (); ++imageIt)
  {
    sprintf (file_name, "gray_%03u.png", index);
    imwrite (file_name, imageIt->second->image);
  }
  //  cout << "write depth images" << endl;
  //  for (imageIt = depth_images_.begin (); imageIt != depth_images_.end (); ++imageIt)
  //  {
  //    sprintf (file_name, "%s_depth_%03d.png", imageIt->first, index);
  //    cout << "saving depth image: " << file_name << endl;
  //    imwrite (file_name, imageIt->second->image);
  //  }

  for (imageIt = rgb_images_.begin (); imageIt != rgb_images_.end (); ++imageIt)
    imageIt->second->lock.unlock ();

  for (imageIt = gray_images_.begin (); imageIt != gray_images_.end (); ++imageIt)
    imageIt->second->lock.unlock ();
  //  for (imageIt = depth_images_.begin (); imageIt != depth_images_.end (); ++imageIt)
  //    imageIt->second->lock.unlock ();
}

void MyOpenNIExample::imageCallback (boost::shared_ptr<Image> image, void* cookie)
{
  timeval timestamp;
  gettimeofday (&timestamp, NULL);

  double now = timestamp.tv_sec + timestamp.tv_usec * 0.000001;
//  double diff1 = min (fabs (now - depth_timestamp), fabs (depth_timestamp - image_timestamp));
//  double diff2 = max (fabs (now - depth_timestamp), fabs (depth_timestamp - image_timestamp));
  //cout << diff1 * 1000.0 << "\tms vs. " << diff2 * 1000.0 << endl;

  image_timestamp = now;
  OpenNIDevice* device = reinterpret_cast<OpenNIDevice*>(cookie);
  ImageContext* rgb_image_context = rgb_images_[device->getConnectionString ()];
  ImageContext* gray_image_context = gray_images_[device->getConnectionString ()];

  // lock image so it does not get drawn
  unique_lock<mutex> rgb_lock (rgb_image_context->lock);
  unsigned char* rgb_buffer = (unsigned char*)(rgb_image_context->image.data + (rgb_image_context->image.cols >> 2) * rgb_image_context->image.elemSize () +
                                              (rgb_image_context->image.rows >> 2) * rgb_image_context->image.step);
  image->fillRGB (rgb_image_context->image.cols >> 1, rgb_image_context->image.rows >> 1, rgb_buffer, rgb_image_context->image.step);
  
/*
  unsigned char* rgb_buffer = (unsigned char*)(rgb_image_context->image.data + (rgb_image_context->image.cols >> 3 ) * 3 * rgb_image_context->image.elemSize () +
                                              (rgb_image_context->image.rows >> 3 ) * 3 * rgb_image_context->image.step);
  image->fillRGB (rgb_image_context->image.cols >> 2, rgb_image_context->image.rows >> 2, rgb_buffer, rgb_image_context->image.step);
*/
  //image->fillRGB (rgb_image_context->image.cols, rgb_image_context->image.rows, rgb_image_context->image.data, rgb_image_context->image.step);
  /*
  cv::Mat raw (image->getHeight(), image->getWidth(), CV_8UC1);
  image->fillRaw (raw.data);

  static int calls = 0;
  if (++calls % 30)
  {
    int index = calls / 30;
    char filename [1024];
    sprintf (filename, "image_%03d.png", index);
    imwrite (filename, raw);
  }
  imshow ("raw", raw);
  */
  rgb_image_context->is_new = true;
  rgb_lock.unlock ();

  unique_lock<mutex> gray_lock (gray_image_context->lock);

  unsigned char* gray_buffer = (unsigned char*)(gray_image_context->image.data + (gray_image_context->image.cols >> 2) +
                                               (gray_image_context->image.rows >> 2) * gray_image_context->image.step);
  image->fillGrayscale (gray_image_context->image.cols >> 1, gray_image_context->image.rows >> 1, gray_buffer, gray_image_context->image.step);
  //image->fillGrayscale (gray_image_context->image.cols, gray_image_context->image.rows, gray_image_context->image.data, gray_image_context->image.step);
  gray_image_context->is_new = true;
}

void MyOpenNIExample::depthCallback (boost::shared_ptr<DepthImage> depth, void* cookie)
{

  timeval timestamp;
  gettimeofday (&timestamp, NULL);
  depth_timestamp = timestamp.tv_sec + timestamp.tv_usec * 0.000001;

  OpenNIDevice* device = reinterpret_cast<OpenNIDevice*>(cookie);
  ImageContext* depth_image_context = depth_images_[device->getConnectionString ()];

  // lock depth image so it does not get drawn
  unique_lock<mutex> depth_lock (depth_image_context->lock);
  float* buffer = (float*)(depth_image_context->image.data + (depth_image_context->image.cols >> 2) * sizeof(float) +
                          (depth_image_context->image.rows >> 2) * depth_image_context->image.step );
  depth->fillDepthImage (depth_image_context->image.cols >> 1, depth_image_context->image.rows >> 1, buffer, depth_image_context->image.step);
  //depth.fillDepthImage (depth_image_context->image.cols, depth_image_context->image.rows, (float*)depth_image_context->image.data, depth_image_context->image.step);
  depth_image_context->is_new = true;
}

int MyOpenNIExample::run ()
{
  running_ = true;
  try
  {
    while (running_)
    {
      for (map<string, ImageContext*>::iterator imageIt = rgb_images_.begin (); imageIt != rgb_images_.end (); ++imageIt)
      {
        if (imageIt->second->is_new && imageIt->second->lock.try_lock ())
        {
          cv::Mat bgr_image;
          cvtColor (imageIt->second->image, bgr_image, CV_RGB2BGR);
          imshow (imageIt->first + "RGB", bgr_image);
          imageIt->second->is_new = false;
          imageIt->second->lock.unlock ();
        }
      }

      for (map<string, ImageContext*>::iterator imageIt = gray_images_.begin (); imageIt != gray_images_.end (); ++imageIt)
      {
        if (imageIt->second->is_new && imageIt->second->lock.try_lock ())
        {
          imshow (imageIt->first + "Gray", imageIt->second->image);
          imageIt->second->is_new = false;
          imageIt->second->lock.unlock ();
        }
      }

      for (map<string, ImageContext*>::iterator imageIt = depth_images_.begin (); imageIt != depth_images_.end (); ++imageIt)
      {
        if (imageIt->second->is_new && imageIt->second->lock.try_lock ())
        {// depth image is in range 0-10 meter -> convert to 0-255 values
          Mat gray_image;
          imageIt->second->image.convertTo (gray_image, CV_8UC1, 25.5);
          imshow (imageIt->first + "Depth", gray_image);
          imageIt->second->is_new = false;
          imageIt->second->lock.unlock ();
        }
      }

      unsigned char key = waitKey (30) & 0xFF;

      switch (key)
      {
        case 27:
        case 'q':
        case 'Q': running_ = false;
          break;

        case '1':
          selected_device_ = 0;
          break;
        case '2':
          selected_device_ = 1;
          break;
        case '3':
          selected_device_ = 2;
          break;

        case 'r':
        case 'R':
          devices_[selected_device_]->setDepthRegistration (!devices_[selected_device_]->isDepthRegistered ());
          break;
        case 's':
        case 'S':
          if (devices_[selected_device_]->isSynchronizationSupported ())
            devices_[selected_device_]->setSynchronization (!devices_[selected_device_]->isSynchronized ());
          break;
        case 'c':
        case 'C':
          if (devices_[selected_device_]->isDepthCropped ())
          {
            depth_images_[devices_[selected_device_]->getConnectionString ()]->lock.lock ();
            //depth_images_[devices_[selected_device_]->getConnectionString ()]->image.create (480, 640, CV_32FC1);
            depth_images_[devices_[selected_device_]->getConnectionString ()]->image.rows = 480;
            depth_images_[devices_[selected_device_]->getConnectionString ()]->image.cols = 640;
            depth_images_[devices_[selected_device_]->getConnectionString ()]->lock.unlock ();
            devices_[selected_device_]->setDepthCropping (0, 0, 0, 0);
          }
          else if (devices_[selected_device_]->isDepthCroppingSupported ())
          {
            depth_images_[devices_[selected_device_]->getConnectionString ()]->lock.lock ();
            //depth_images_[devices_[selected_device_]->getConnectionString ()]->image.create (300, 400, CV_32FC1);
            depth_images_[devices_[selected_device_]->getConnectionString ()]->image.rows = 240;
            depth_images_[devices_[selected_device_]->getConnectionString ()]->image.cols = 320;
            depth_images_[devices_[selected_device_]->getConnectionString ()]->lock.unlock ();
            devices_[selected_device_]->setDepthCropping (100, 100, 320, 240);
          }
          break;

        case 'd':
        case 'D':
          if (devices_[selected_device_]->isDepthStreamRunning ())
            devices_[selected_device_]->stopDepthStream ();
          else
            devices_[selected_device_]->startDepthStream ();
          break;
        case 'i':
        case 'I':
          if (devices_[selected_device_]->isImageStreamRunning ())
            devices_[selected_device_]->stopImageStream ();
          else
            devices_[selected_device_]->startImageStream ();
          break;

        case 'w':
        case 'W':
          writeImages ();
          break;
      }
    }
  }
  catch (const OpenNIException& exception)
  {
    cout << "exception caught: " << exception.what () << endl;
    return (-1);
  }
  catch (...)
  {

    cout << "unknown exception caught" << endl;
    return (-1);
  }
  return 0;
}

int main (int argc, char** argv)
{
  OpenNIDriver& driver = OpenNIDriver::getInstance ();
  if (argc == 1)
  {
    cout << "Usage: " << argv[0] << " (<device-index>)+" << endl;
    if (driver.getNumberDevices () > 0)
    {
      for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
      {
        cout << "Device: " << deviceIdx << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: " << driver.getProductName (deviceIdx)
                << ", connected: " << (int)driver.getBus (deviceIdx) << " @ " << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
      }
    }
    else
      cout << "No devices connected." << endl;
    exit (1);
  }

  vector <unsigned> device_indices;
  for (int argIdx = 1; argIdx < argc; ++argIdx)
  {
    unsigned deviceIdx = (unsigned)atoi (argv[argIdx]);
    if (deviceIdx >= driver.getNumberDevices ())
    {
      if (driver.getNumberDevices () > 0)
      {
        cout << "Device index out of range. " << driver.getNumberDevices () << " devices found." << endl;
        for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices (); ++deviceIdx)
        {
          cout << "Device: " << deviceIdx << ", vendor: " << driver.getVendorName (deviceIdx) << ", product: "
                  << driver.getProductName (deviceIdx) << ", connected: " << (int)driver.getBus (deviceIdx) << " @ "
                  << (int)driver.getAddress (deviceIdx) << ", serial number: \'" << driver.getSerialNumber (deviceIdx) << "\'" << endl;
        }
      }
      else
        cout << "No devices connected." << endl;
      exit (-1);
    }
    device_indices.push_back ((unsigned)deviceIdx);
  }

  cout << "<1,2,3...> to select device" << endl;
  cout << "<I> to start or stop image stream of selected device" << endl;
  cout << "<D> to start or stop depth stream of selected device" << endl;
  cout << "<R> to turn on or off registration for selected device" << endl;
  cout << "<S> to turn on or off synchronization for selected device" << endl;
  cout << "<C> to turn on or off image cropping for selected device" << endl;
  cout << "<W> write current images" << endl;
  cout << "<Q> to quit application" << endl;
  MyOpenNIExample example (device_indices);

  return example.run ();
}
