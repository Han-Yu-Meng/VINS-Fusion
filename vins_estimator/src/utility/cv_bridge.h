#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <std_msgs/Header.h>

namespace cv_bridge
{

class CvImage
{
public:
    std_msgs::Header header;
    std::string encoding;
    cv::Mat image;

    CvImage() {}
    CvImage(const std_msgs::Header& _header, const std::string& _encoding, const cv::Mat& _image)
        : header(_header), encoding(_encoding), image(_image) {}

    sensor_msgs::ImagePtr toImageMsg() const
    {
        sensor_msgs::ImagePtr ptr(new sensor_msgs::Image());
        ptr->header = header;
        ptr->encoding = encoding;
        ptr->height = image.rows;
        ptr->width = image.cols;

        if (encoding == sensor_msgs::image_encodings::MONO8 || encoding == "8UC1")
        {
            ptr->step = image.cols;
            ptr->data.assign(image.data, image.data + ptr->step * ptr->height);
        }
        else if (encoding == sensor_msgs::image_encodings::BGR8 || encoding == "8UC3")
        {
            ptr->step = image.cols * 3;
            ptr->data.assign(image.data, image.data + ptr->step * ptr->height);
        }
        else if (encoding == sensor_msgs::image_encodings::BGRA8 || encoding == "8UC4")
        {
            ptr->step = image.cols * 4;
            ptr->data.assign(image.data, image.data + ptr->step * ptr->height);
        }
        else
        {
            ROS_ERROR("cv_bridge simplified: unsupported encoding %s", encoding.c_str());
        }
        return ptr;
    }
};

typedef boost::shared_ptr<CvImage> CvImagePtr;
typedef boost::shared_ptr<const CvImage> CvImageConstPtr;

inline CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source, const std::string& encoding = std::string())
{
    CvImagePtr ptr(new CvImage());
    ptr->header = source->header;
    
    int type;
    if (source->encoding == sensor_msgs::image_encodings::MONO8 || source->encoding == "8UC1")
    {
        type = CV_8UC1;
    }
    else if (source->encoding == sensor_msgs::image_encodings::BGR8 || source->encoding == "8UC3")
    {
        type = CV_8UC3;
    }
    else if (source->encoding == sensor_msgs::image_encodings::BGRA8 || source->encoding == "8UC4")
    {
        type = CV_8UC4;
    }
    else
    {
        ROS_ERROR("cv_bridge simplified: unsupported source encoding %s", source->encoding.c_str());
        return nullptr;
    }

    ptr->image = cv::Mat(source->height, source->width, type, const_cast<uchar*>(&source->data[0])).clone();
    ptr->encoding = source->encoding;

    if (!encoding.empty() && encoding != source->encoding)
    {
        if (encoding == sensor_msgs::image_encodings::MONO8 && (source->encoding == sensor_msgs::image_encodings::BGR8 || source->encoding == "8UC3"))
        {
            cv::cvtColor(ptr->image, ptr->image, cv::COLOR_BGR2GRAY);
            ptr->encoding = encoding;
        }
        else
        {
             ROS_ERROR("cv_bridge simplified: conversion from %s to %s not implemented", source->encoding.c_str(), encoding.c_str());
        }
    }
    return ptr;
}

inline CvImagePtr toCvCopy(const sensor_msgs::Image& source, const std::string& encoding = std::string())
{
    return toCvCopy(sensor_msgs::ImageConstPtr(new sensor_msgs::Image(source)), encoding);
}

}
