#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

double DEG2RAD(double deg) { return deg * MY_PI / 180.0; }

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
        cos(theta), 0, sin(theta), 0, 
        0, 1, 0, 0,
        -sin(theta), 0,  cos(theta), 0,
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


Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
    // Assume that the light positions are in the view space
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    // The eye_pos in the view space is at the coordinate origin
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f I_RR = light.intensity / (light.position - point).squaredNorm();
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        Eigen::Vector3f diffuse = kd.cwiseProduct(I_RR) * std::max(0.f, light_dir.dot(normal));
        
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(I_RR) * std::pow(std::max(0.f, half_dir.dot(normal)), p);

        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color+= diffuse + specular + ambient;
    }

    return result_color * 255.f;
}


Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        return_color = payload.texture->getColor(payload.tex_coords[0],payload.tex_coords[1]);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
    // Assume that the light positions are in the view space
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    // The eye_pos in the view space is at the coordinate origin
    Eigen::Vector3f eye_pos{0, 0, 0};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    
    Eigen::Vector3f result_color = {0, 0, 0};
    
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f I_RR = light.intensity / (light.position - point).squaredNorm();
        Eigen::Vector3f light_dir = (light.position - point).normalized();
        Eigen::Vector3f view_dir = (eye_pos - point).normalized();
        Eigen::Vector3f diffuse = kd.cwiseProduct(I_RR) * std::max(0.f, light_dir.dot(normal));
        
        Eigen::Vector3f half_dir = (light_dir + view_dir).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(I_RR) * std::pow(std::max(0.f, half_dir.dot(normal)), p);

        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color+= diffuse + specular + ambient;
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    Eigen::Vector3f n = normal;
    Eigen::Matrix3f TBN;
    
    // float x = normal.x();
    // float y = normal.y();
    // float z = normal.z();
    // Eigen::Vector3f t = {x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z)};
    // Eigen::Vector3f b = n.cross(t);
    // TBN<< t,b,n;
    
    TBN = payload.TBN;
    float dU = kh * kn * (payload.texture->getColor(payload.tex_coords.x()+ 1.f/payload.texture->width,payload.tex_coords.y()).norm() -payload.texture->getColor(payload.tex_coords.x(),payload.tex_coords.y()).norm());
    float dV = kh * kn * (payload.texture->getColor(payload.tex_coords.x(),payload.tex_coords.y() + + 1.f/payload.texture->height).norm() -payload.texture->getColor(payload.tex_coords.x(),payload.tex_coords.y()).norm());
    Eigen::Vector3f ln = {-dU, -dV, 1.f};
    n = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = n;
    
    return result_color * 255.f;
}


Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.


    }

    return result_color * 255.f;
}


int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;
    float angle_z = 140;// The angle rotate around z-axis
    Eigen::Vector3f any_axis = {0,2,-2};// The angle rotate around any axis passing through the origin
    any_axis.normalize();
    float any_angle = 0;// The angle rotate around z-axis
    float x_pos = 0;
    float size = 2.5f;

    
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";
    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }
    
    
    // Screen resolution 700x700
    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }
    
    // Camera coordinate
    Eigen::Vector3f eye_pos = {0, 0, 10};
    Eigen::Vector3f eye_lookat = {0,0,-1};
    Eigen::Vector3f eye_up = {0,1,0};
    
    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);
    
    // 输入的指令字符
    int key = 0;
    // 运行的帧数
    int frame_count = 0;
    
    // 若存在命令行参数，则直接运行一次输出image即结束
    if (command_line) {
        // 清空color和depth的framebuffer
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        // 设置MVP矩阵
        r.set_model(get_model_matrix(angle_z, x_pos, size, any_angle,any_axis));
        r.set_view(get_view_matrix(eye_pos, eye_lookat, eye_up));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));
        
        r.draw(TriangleList);
        
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);
        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle_z, x_pos, size, any_angle,any_axis));
        r.set_view(get_view_matrix(eye_pos, eye_lookat, eye_up));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));
        
        r.draw(TriangleList);
        
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);
        std::cout << "frame count: " << frame_count++ << '\n';
        
        // Rotate the camera around the Z-axis
        if (key == 'a') {
            angle_z += 10;
        }
        else if (key == 'd') {
            angle_z -= 10;
        }
        // Rotate the camera around the Z-axis
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
