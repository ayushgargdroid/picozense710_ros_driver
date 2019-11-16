#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/distortion_models.h>
#include <tf/transform_broadcaster.h>

// comment out strings if need not needed
#include <string>
#include <cerrno>
#include <iostream>
#include <stdint.h>
#include <ctype.h>
#include <vector>
#include <fstream>

#include "../dependencies/PicoZenseSDK_DCAM710/Include/PicoZense_api.h"

#define MAX_PATH_SIZE 1024

using namespace std;
using std::cout;
using std::cin;
using namespace cv;

sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t,int seqnum) {
  sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  sensor_msgs::Image& imgMessage = *ptr;
  imgMessage.header.stamp = t;
  imgMessage.header.seq = seqnum;
  imgMessage.header.frame_id = frameId;
  imgMessage.height = img.rows;
  imgMessage.width = img.cols;
  imgMessage.encoding = encodingType;
  int num = 1; //for endianness detection
  imgMessage.is_bigendian = !(*(char *) &num == 1);
  imgMessage.step = img.cols * img.elemSize();
  size_t size = imgMessage.step * img.rows;
  imgMessage.data.resize(size);

  if (img.isContinuous())
    memcpy((char*) (&imgMessage.data[0]), img.data, size);
  else {
    uchar* opencvData = img.data;
    uchar* rosData = (uchar*) (&imgMessage.data[0]);
    for (unsigned int i = 0; i < img.rows; i++) {
      memcpy(rosData, opencvData, imgMessage.step);
      rosData += imgMessage.step;
      opencvData += img.step;
    }
  }
  return ptr;
}

void PicofillCamInfo(int _deviceIndex,sensor_msgs::CameraInfoPtr pico_cam_info_msg){

  pico_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  
  uint16_t resolution;
  PsCameraParameters params;

  PsReturnStatus status = PsGetResolution(_deviceIndex,&resolution);
  if(status == PsRetOK)
  {
    switch (resolution)
    {
    case PsRGB_Resolution_1920_1080:
      pico_cam_info_msg->width = 1920;
      pico_cam_info_msg->height = 1080;
      break;
    
    case PsRGB_Resolution_1280_720:
      pico_cam_info_msg->width = 1280;
      pico_cam_info_msg->height = 720;
      break;

    case PsRGB_Resolution_640_480:
      pico_cam_info_msg->width = 640;
      pico_cam_info_msg->height = 480;
      break;

    case PsRGB_Resolution_640_360:
      pico_cam_info_msg->width = 640;
      pico_cam_info_msg->height = 360;
      break;
    }
  }

  status = PsGetCameraParameters(_deviceIndex,PsSensorType::PsDepthSensor,&params);

  pico_cam_info_msg->D.resize(5);
  pico_cam_info_msg->D[0] = params.k1;   // k1
  pico_cam_info_msg->D[1] = params.k2;   // k2
  pico_cam_info_msg->D[2] = params.k3;   // k3
  pico_cam_info_msg->D[3] = params.p1;   // p1
  pico_cam_info_msg->D[4] = params.p2;   // p2

  pico_cam_info_msg->K.fill(0.0);
  pico_cam_info_msg->K[0] = params.fx;
  pico_cam_info_msg->K[2] = params.cx;
  pico_cam_info_msg->K[4] = params.fy;
  pico_cam_info_msg->K[5] = params.cy;
  pico_cam_info_msg->K[8] = 1.0;

  pico_cam_info_msg->R.fill(0.0);

  for (int i = 0; i < 3; i++) {
    // identity
    pico_cam_info_msg->R[i + i * 3] = 1;
  }

  pico_cam_info_msg->P.fill(0.0);
  pico_cam_info_msg->P[0] = params.fx;
  pico_cam_info_msg->P[2] = params.cx;
  pico_cam_info_msg->P[5] = params.fy;
  pico_cam_info_msg->P[6] = params.cy;
  pico_cam_info_msg->P[10] = 1.0;

  pico_cam_info_msg->header.frame_id = "picoCam";

}

void publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t) {
  static int seq = 0;
  camInfoMsg->header.stamp = t;
  camInfoMsg->header.seq = seq;
  pubCamInfo.publish(camInfoMsg);
  seq++;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  //creating publisher for image through ImageTransport
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  // image_transport::Publisher pub = it.advertise("image_rect", 1);   // #looks better on aarch64 #depending on whether laser scan or point cloud, change the topic to image or image_rect respectively


  /* Picozense API calls from this point onwards */
  //PsReturnStatus status;
  bool colorMap = true;
  int32_t deviceIndex = 0;
  uint32_t slope = 3000;//4400;//1450;  // Near mode
  uint16_t threshold = 10;

  PsReturnStatus status = PsInitialize();
  ROS_INFO_STREAM( "Init status: " << status << "\n");

  if (status != PsReturnStatus::PsRetOK)
  {
      ROS_INFO_STREAM("Initialize failed!" << "\n");
      system("pause");
      exit(0);
  }


  //publisher for camera information
  // aarch64 only
  ros::Publisher pubPicoCamInfo = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  int32_t deviceCount = 0;
  status = PsGetDeviceCount(&deviceCount);
  ROS_INFO_STREAM( "Get device count status: " << status << " device count: " << deviceCount << "\n");

  status = PsSetThreshold(deviceIndex, threshold);
  cout << "Set threshold status: " << status << endl;

  //Set the Depth Range to Mid through PsSetDepthRange interface
  // PsNearRange, PsMidRange, PsFarRange
  status = PsSetDepthRange(deviceIndex, PsNearRange);
  ROS_INFO_STREAM( "Set depth range status: " << status << "\n");

  status = PsOpenDevice(deviceIndex);

  if (status != PsReturnStatus::PsRetOK)
  {
    ROS_INFO_STREAM ("OpenDevice failed!" << "\n");
    system("pause");
    exit(0);
  }

  int32_t depthRange = -1;
  status = PsGetDepthRange(deviceIndex, (PsDepthRange*)&depthRange);
  ROS_INFO_STREAM( "Get depth range status: " << status << " depth range: " << depthRange << "\n");

  status = PsStartFrame(deviceIndex, PsDepthFrame);
  ROS_INFO_STREAM( "Start Depth Frame status: " << status << "\n");

  // status = PsStartFrame(deviceIndex, PsIRFrame);
  // ROS_INFO_STREAM( "Start IR Frame status: " << status << "\n");

  PsFrameMode depthFrameMode;
  // PsFrameMode irFrameMode;

  status = PsGetFrameMode(deviceIndex, PsDepthFrame, &depthFrameMode);
  ROS_INFO_STREAM( "Get Depth Frame mode status: " << status << "\n");
  ROS_INFO_STREAM( "depthFrameMode.pixelFormat: " << depthFrameMode.pixelFormat << "\n");
  ROS_INFO_STREAM( "depthFrameMode.resolutionWidth: " << depthFrameMode.resolutionWidth << "\n");
  ROS_INFO_STREAM( "depthFrameMode.resolutionHeight: " << depthFrameMode.resolutionHeight << "\n");
  ROS_INFO_STREAM( "depthFrameMode.fps: " << depthFrameMode.fps << "\n");

  PsCameraParameters cameraParameters;
  status = PsGetCameraParameters(deviceIndex, PsDepthSensor, &cameraParameters);

  ROS_INFO_STREAM( "Get PsGetCameraParameters status: " << status << "\n");
  ROS_INFO_STREAM( "Camera Intinsic: " << "\n");
  ROS_INFO_STREAM( "Fx: " << cameraParameters.fx << "\n");
  ROS_INFO_STREAM( "Cx: " << cameraParameters.cx << "\n");
  ROS_INFO_STREAM( "Fy: " << cameraParameters.fy << "\n");
  ROS_INFO_STREAM( "Cy: " << cameraParameters.cy << "\n");
  ROS_INFO_STREAM( "Distortion Coefficient: " << "\n");
  ROS_INFO_STREAM( "K1: " << cameraParameters.k1 << "\n");
  ROS_INFO_STREAM( "K2: " << cameraParameters.k2 << "\n");
  ROS_INFO_STREAM( "K3: " << cameraParameters.k3 << "\n");
  ROS_INFO_STREAM( "P1: " << cameraParameters.p1 << "\n");
  ROS_INFO_STREAM( "P2: " << cameraParameters.p2 << "\n");

  cv::Mat mDispImg;
  cv::Mat mDispImgdiv16; // Image (RAW12 representation converted to 16 bit)
  sensor_msgs::CameraInfoPtr pico_cam_info_msg_(new sensor_msgs::CameraInfo());

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  //creating a camera info message to be published
  PicofillCamInfo(deviceIndex,pico_cam_info_msg_);
  /* Transform message construction. Values are set to 0. Update the values using the position of camera from the base */
  transform.setOrigin( tf::Vector3(0, 0, 0.25) ); // position with respect to base (in meters)
  q.setRPY(0, 0, 0); // describes rotation with respect to base
  transform.setRotation(q);

  /* transforms */
  ros::Rate loop_rate(1);
  int counter = 0;

  while (nh.ok()) {

    PsReadNextFrame(deviceIndex);

    PsFrame depthFrame;

    PsReturnStatus status = PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);

    if (status == PsRetOK) {
      mDispImg = cv::Mat(depthFrameMode.resolutionHeight, depthFrameMode.resolutionWidth, CV_16UC1, depthFrame.pFrameData);
      mDispImgdiv16 = mDispImg*16;
    }

    ros::Time present_time = ros::Time::now();

    sensor_msgs::ImagePtr msg = imageToROSmsg(mDispImgdiv16, "16UC1", "camera_depth_frame" , present_time, counter);

    counter++;

    ROS_INFO_STREAM("  present_time " <<present_time.nsec);

    pub.publish(msg);

    publishCamInfo(pico_cam_info_msg_,pubPicoCamInfo,present_time);

    mDispImg.release();
    ros::spinOnce();
  }

  status = PsStopFrame(deviceIndex, PsDepthFrame);
  ROS_INFO_STREAM( "Stop Depth Frame status: " << status << "\n");

  status = PsCloseDevice(deviceIndex);
  ROS_INFO_STREAM( "CloseDevice status: " << status << "\n");

  status = PsShutdown();
  ROS_INFO_STREAM( "Shutdown status: " << status << "\n");

}