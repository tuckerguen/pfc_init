#ifndef NEEDLE_IMAGE_H
#define NEEDLE_IMAGE_H

#include <opencv2/core.hpp>
#include "TemplateMatch.hpp"
#include "NeedleTemplate.hpp"
#include <string>

class NeedleImage
{
public:
    cv::Mat raw;
    cv::Mat image;  
    std::string path;
    explicit NeedleImage(std::string path);
};

#endif