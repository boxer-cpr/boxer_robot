/**
Software License Agreement (BSD)

\file      camera_focus_node.cpp
\authors   Vaibhav Mehta <vmehta@clearpathrobotics.com>
\copyright Copyright (c) 2016, Rockwell Automation Technologies, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Rockwell Automation Technologies nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>

namespace camera_focus
{

class CameraFocus
{
public:
  CameraFocus()
    : method_(0)
    , max_high_freq_measure_(0.0)
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    nh_priv.param("method", method_, method_);

    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe("image", 1, &CameraFocus::imageCallback, this);

    cv::namedWindow("focus_viewer");
    cv::startWindowThread();
  }

  ~CameraFocus()
  {
    cv::destroyWindow("focus_viewer");
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat out;
    double high_freq_measure;
    if (method_ == 0)
    {
      high_freq_measure = sobelMetric(cv_ptr->image);
    }
    else if (method_ == 1)
    {
      high_freq_measure = dftMetric(cv_ptr->image);
    }

    if (high_freq_measure > max_high_freq_measure_)
    {
      max_high_freq_measure_ = high_freq_measure;
    }

    std::ostringstream ss;
    ss << high_freq_measure << "/" <<max_high_freq_measure_;
    std::string s(ss.str());

    float threshold = 10.0;

    if (high_freq_measure > 0.95*max_high_freq_measure_)
    {
      cv::rectangle(cv_ptr->image, cv::Point(0, 0), cv::Point(cv_ptr->image.cols-10+1, cv_ptr->image.rows-10+1),
                    cv::Scalar(0, 255, 0), 10);
    }
    else
    {
      cv::rectangle(cv_ptr->image, cv::Point(0, 0), cv::Point(cv_ptr->image.cols-10+1, cv_ptr->image.rows-10+1),
                    cv::Scalar(0, 0, 255), 10);
    }
    cv::putText(cv_ptr->image, s, cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0), 3);

    // Update GUI Window
    cv::imshow("focus_viewer", cv_ptr->image);
    cv::waitKey(1);
  }

  double sobelMetric(const cv::Mat& image_input)
  {
    cv::Mat dx, dy, out;
    cv::Sobel(image_input, dx, CV_32F, 1, 0, 3);
    cv::Sobel(image_input, dy, CV_32F, 0, 1, 3);

    magnitude(dx, dy, out);
    return sum(out)[0]/1e+06;
  }

  double dftMetric(const cv::Mat& image_input)
  {
    cv::Mat img_gray, img_float32, img_dft, out;
    cv::cvtColor(image_input, img_gray, CV_BGR2GRAY);
    // assuming grey scale input
    img_gray.convertTo(img_float32, CV_32FC1);

    cv::Mat padded;                            // expand input image to optimal size
    int m = cv::getOptimalDFTSize(img_float32.rows);
    int n = cv::getOptimalDFTSize(img_float32.cols);  // on the border add zero values
    copyMakeBorder(img_float32, padded, 0, m - img_float32.rows, 0, n - img_float32.cols,
                    cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))

    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))

    magnitude(planes[0], planes[1], out);
    return sum(out)[0]/1e+08;
  }

private:
  image_transport::Subscriber image_sub_;
  int method_;
  double max_high_freq_measure_;
};

}  // namespace camera_focus

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_focus_node");
  camera_focus::CameraFocus cf_viewer;

  ros::spin();
  return EXIT_SUCCESS;
}
