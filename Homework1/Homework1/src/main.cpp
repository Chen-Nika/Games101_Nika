#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

inline double DEG2RAD(double deg) { return deg * EIGEN_PI / 180.0; }

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f lookat, Eigen::Vector3f up)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    // Move the camera to the origin
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;
    // Rotate the camera to align with the world space coordinate axes
    Eigen::Matrix4f rotation;
    // The current z axe of the camera
    Eigen::Vector3f z = -lookat;
    // The current y axe of the camera
    Eigen::Vector3f y = up;
    // The current x axe of the camera, -Z x Y (gxt in games101 slide) or Y x Z 
    Eigen::Vector3f x = lookat.cross(up);
    rotation <<
        x[0], y[0], z[0], 0,
        x[1], y[1], z[1], 0,
        x[2], y[2], z[2], 0,
        0,    0,    0,    1;
    
    view = rotation * translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, float x_pos, float size, float rotation_angle_any, Eigen::Vector3f any_axis)
{
    Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // Uniform scale
    Eigen::Matrix4f scale_matrix = Eigen::Matrix4f::Identity();
    scale_matrix <<
        size, 0, 0, 0,
        0, size, 0, 0,
        0, 0, size, 0,
        0, 0, 0, 1;
    
    // Rotate around the Z-axis
    Eigen::Matrix4f rotate_matrix = Eigen::Matrix4f::Identity();
    // Angle to radian
    float theta = DEG2RAD(rotation_angle);
    rotate_matrix <<
        cos(theta), -sin(theta), 0, 0,
        sin(theta), cos(theta), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Rotate around any axis passing through the origin
    Eigen::Matrix4f rotate_any_matrix = Eigen::Matrix4f::Identity();
    // Angle to radian
    float alpha = DEG2RAD(rotation_angle_any);
    Eigen::Vector3f n = any_axis;
    Eigen::Matrix3f N;
    N<<
        0,-n.z(),n.y(),
        n.z(),0,-n.x(),
        -n.y(),n.x(),0;
    Eigen::Matrix3f R = cos(alpha) * Eigen::Matrix3f::Identity() + (1-cos(alpha))*n*n.transpose() + sin(alpha) * N;
    rotate_any_matrix<<
        R(0,0), R(0,1), R(0,2), 0,
        R(1,0), R(1,1), R(1,2), 0,
        R(2,0), R(2,1), R(2,2), 0,
        0,0,0,1;
    // Another way to call the Eigen library
    // AngleAxisf rot(rotation_angle * 3.14159265f / 180.0f, any_axis.normalized());
    // Isometry3f tt = Isometry3f::Identity();
    // rotate_any_matrix = tt.rotate(rot).matrix();
    
    // Move along the x-axis
    Eigen::Matrix4f translate_matrix = Eigen::Matrix4f::Identity();
    translate_matrix <<
        1, 0, 0, x_pos,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    
    
    model_matrix = translate_matrix * rotate_any_matrix * rotate_matrix * scale_matrix * model_matrix;
    
    return model_matrix;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection;
    float top = -tan(DEG2RAD(eye_fov / 2.0f) * abs(zNear));
    float right = -top * aspect_ratio;

    projection << zNear / right, 0, 0, 0,
        0, -zNear / top, 0, 0,
        0, 0, (zNear + zFar) / (zNear - zFar), (-2 * zNear * zFar) / (zNear - zFar),
        0, 0, 1, 0;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle_z = 0;// The angle rotate around z-axis
    Eigen::Vector3f any_axis = {0,2,-2};// The angle rotate around any axis passing through the origin
    any_axis.normalize();
    float any_angle = 0;// The angle rotate around z-axis
    float x_pos = 0;
    float size = 1.0f;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        // Parse the angle to rotate
        angle_z = std::stof(argv[2]); // -r by default
        // Parse the filename to write
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }
    // Screen resolution 700x700
    rst::rasterizer r(700, 700);
    // Camera coordinate
    Eigen::Vector3f eye_pos = {0, 0, 15};
    Eigen::Vector3f eye_lookat = {0,0,-1};
    Eigen::Vector3f eye_up = {0,1,0};
    // The three vertices of the triangle, in counterclockwise direction
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
    // The index ids of the vertices
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
        r.set_model(get_model_matrix(angle_z, x_pos, size,any_angle,any_axis));
        r.set_view(get_view_matrix(eye_pos, eye_lookat, eye_up));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle_z, x_pos, size, any_angle,any_axis));
        r.set_view(get_view_matrix(eye_pos, eye_lookat, eye_up));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle_z += 10;
        }
        else if (key == 'd') {
            angle_z -= 10;
        }
        else if (key == 'o') {
            any_angle -= 10;
        }
        else if (key == 'p') {
            any_angle += 10;
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
