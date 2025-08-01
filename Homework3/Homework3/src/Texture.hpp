//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        int u_img = std::clamp(static_cast<int>(u * width), 0, width - 1);
        int v_img = std::clamp(static_cast<int>((1 - v) * height), 0, height - 1);;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f lerpColor(float l, Eigen::Vector3f color0, Eigen::Vector3f color1)
    {
        return color0 + (color1 - color0) * l;
    }

    Eigen::Vector3f getColorImg(float u_img, float v_img)
    {
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = std::clamp((u * width), 0.f, (float)width);
        float v_img = std::clamp((1.f - v) * height, 0.f, (float)height);
        
        float u0 = std::max(std::floor(u_img - 0.5f), 0.f);
        float u1 = std::min(u0 + 1.f, width - 1.f);
        float v0 = std::max(std::floor(v_img - 0.5f), 0.f);
        float v1 = std::min(v0 + 1.f, height - 1.f);

        float s = u_img - u0 - 0.5f;
        float t = v_img - v0 - 0.5f;
        
        Eigen::Vector3f color0 = lerpColor(s,getColorImg(u0, v0), getColorImg(u1, v0));
        Eigen::Vector3f color1 = lerpColor(s,getColorImg(u0, v1), getColorImg(u1, v1));
        return lerpColor(t, color0, color1);
    }

};
#endif //RASTERIZER_TEXTURE_H
