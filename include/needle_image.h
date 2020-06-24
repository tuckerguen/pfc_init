#ifndef NEEDLE_IMAGE_H
#define NEEDLE_IMAGE_H

#include <opencv2/core.hpp>

/**
 *  @brief Respresents one endoscope image of the needle 
 */
class NeedleImage
{
public:
    /**
     * @brief the original image before preprocessing
     */
    cv::Mat raw;

    /**
     * @brief the image after preprocessing (used for template matching)
     */
    cv::Mat image;  

    /**
     * @brief path to the image
     */
    std::string path;

    /**
     * @brief filters needle from the background in the raw image 
     */
    void filterRaw();
    
    /**
     * @brief the explicit constructor
     * 
     * @param path Path to the image 
     */
    explicit NeedleImage(const std::string path);

    /**
     * @brief the deconstructor
     */
    ~NeedleImage()
    {};
};

#endif