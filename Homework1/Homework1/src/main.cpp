#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

inline double DEG2RAD(double deg) { return deg * EIGEN_PI / 180.0; }

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, float x_pos, float size)
{
    Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // 整体缩放
    Eigen::Matrix4f scale_matrix = Eigen::Matrix4f::Identity();
    scale_matrix <<
        size, 0, 0, 0,
        0, size, 0, 0,
        0, 0, size, 0,
        0, 0, 0, 1;
    
    // 绕Z轴旋转
    Eigen::Matrix4f rotate_matrix = Eigen::Matrix4f::Identity();
    // 转弧度
    float theta = DEG2RAD(rotation_angle);
    rotate_matrix <<
        cos(theta), -sin(theta), 0, 0,
        sin(theta), cos(theta), 0, 0,
        0, 0, 1, 1,
        0, 0, 0, 1;

    // 沿着x轴移动
    Eigen::Matrix4f translate_matrix = Eigen::Matrix4f::Identity();
    translate_matrix <<
        1, 0, 0, x_pos,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    
    model_matrix = translate_matrix * rotate_matrix * scale_matrix * model_matrix;
    return model_matrix;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection;
    float top = -tan(DEG2RAD(eye_fov / 2.0f) * abs(zNear));
    float right = top * aspect_ratio;

    projection << zNear / right, 0, 0, 0,
        0, zNear / top, 0, 0,
        0, 0, (zNear + zFar) / (zNear - zFar), (2 * zNear * zFar) / (zFar - zNear),
        0, 0, 1, 0;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    float x_pos = 0;
    float size = 1.0f;
    // 是否有命令行参数
    bool command_line = false;
    // 输出名
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        // 旋转的角度
        angle = std::stof(argv[2]); // -r by default
        // 输出的图片文件名
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }
    // 屏幕分辨率是700x700
    rst::rasterizer r(700, 700);
    // 相机坐标
    Eigen::Vector3f eye_pos = {0, 0, 5};
    // 三角形的3个顶点 逆时针分布
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    // 3个顶点对应的索引id
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};
    // 将顶点坐标和索引id都存入各自的存储map
    // 每次调用都对id+1,用于辨别缓存id
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    // 输入的指令字符
    int key = 0;
    // 运行的帧数
    int frame_count = 0;
    
    // 若存在命令行参数，则直接运行一次输出image即结束
    if (command_line) {
        // 清空color和depth的framebuffer
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        // 设置MVP矩阵
        r.set_model(get_model_matrix(angle, x_pos, size));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, x_pos, size));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'l') {
            x_pos -= 1;
        }
        else if (key == 'r') {
            x_pos += 1;
        }
        else if (key == 'b') {
            size += 0.1f;
            size = std::max(size, 0.1f);
        }
        else if (key == 's') {
            size -= 0.1f;
            size = std::max(size, 0.1f);
        }
    }

    return 0;
}
