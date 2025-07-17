// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}


static bool insideTriangle(int x, int y, const Vector4f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // Reference: https://www.scratchapixel.com/lessons/3d-basic-rendering/rasterization-practical-implementation/rasterization-stage.html
    // Inside the triangle?
    //  1. Is the cross product of two vectors greater than 0.0f
    //  2. If cross product = 0,
    //      - At Left / Top edge, YES
    //  2. If cross product > 0, YES
    Vector3f edge0 = (_v[1] - _v[0]).head<3>();
    Vector3f edge1 = (_v[2] - _v[1]).head<3>();
    Vector3f edge2 = (_v[0] - _v[2]).head<3>();
    float w0,w1,w2;
    // Cross product
    w0 = edge0.x() * (y - _v[0].y()) - edge0.y() * (x - _v[0].x());
    w1 = edge1.x() * (y - _v[1].y()) - edge1.y() * (x - _v[1].x());
    w2 = edge2.x() * (y - _v[2].y()) - edge2.y() * (x - _v[2].x());
    bool inside = true;
    inside &= (w0 == 0.f ? ((edge0.y() == 0.f && edge0.x() <0.f) || (edge0.y() < 0.f)) : (w0 > 0.f));
    inside &= (w1 == 0.f ? ((edge1.y() == 0.f && edge1.x() <0.f) || (edge1.y() < 0.f)) : (w1 > 0.f));
    inside &= (w2 == 0.f ? ((edge2.y() == 0.f && edge2.x() <0.f) || (edge2.y() < 0.f)) : (w2 > 0.f));
    return inside;
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    // Each element of the ind_buf is an array, representing the three vertex index IDs of a triangle
    for (auto& i : ind)
    {
        Triangle t;
        // transform to clipping space
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            // Keep the z value in viewspace
            float tmpW = vec.w();
            vec /= vec.w();
            vec.w() = tmpW;
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }
        // The vertices of t are in the screen space
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i]);
            t.setVertex(i, v[i]);
            t.setVertex(i, v[i]);
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];
        
        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
        
        // rasterize_triangle_noAA(t);
        //rasterize_triangle_SSAA(t);
        rasterize_triangle_MSAA(t);
    }
}


//Screen space rasterization
void rst::rasterizer::rasterize_triangle_noAA(const Triangle& t) {
    auto v = t.toVector4();
    // calculate bounding box
    float minX,maxX,minY,maxY;
    minX = maxX = v[0].x();
    minY = maxY = v[0].y();
    for (int i = 1; i < 3; ++i)
    {
        minX = std::min(minX, v[i].x());
        minY = std::min(minY, v[i].y());
        maxX = std::max(maxX, v[i].x());
        maxY = std::max(maxY, v[i].y());
    }

    for (int y= std::floor(minY); y < std::ceil(maxY); y++)
        for (int x= std::floor(minX); x < std::ceil(maxX); x++)
        {
            // No anti-aliasing
            if (insideTriangle(x,y,v.data()))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5f, y+0.5f, v.data());
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                int buf_idx = get_index(x,y);
                if (w_reciprocal > depth_buf[buf_idx])
                {
                    set_pixel(Vector3f(x, y, 0), t.getColor());
                    depth_buf[buf_idx] = w_reciprocal;
                }
            }
        }
}

void rst::rasterizer::rasterize_triangle_SSAA(const Triangle& t)
{
    auto v = t.toVector4();
    // calculate bounding box
    float minX,maxX,minY,maxY;
    minX = maxX = v[0].x();
    minY = maxY = v[0].y();
    for (int i = 1; i < 3; ++i)
    {
        minX = std::min(minX, v[i].x());
        minY = std::min(minY, v[i].y());
        maxX = std::max(maxX, v[i].x());
        maxY = std::max(maxY, v[i].y());
    }
    
    Vector2f offsets[4] = {{0.25f, 0.25f},{0.25f, 0.75f},{0.75f, 0.25f},{0.75f, 0.75f}}; 
    for (int y= std::floor(minY); y < std::ceil(maxY); y++)
        for (int x= std::floor(minX); x < std::ceil(maxX); x++)
        {
            int eid = get_index(x,y)* AAFactor;
            // SuperSampling Anti-Aliasing
            for (int sp=0; sp<AAFactor; ++sp)
            {
                float xSample = x + offsets[sp].x();
                float ySample = y + offsets[sp].y();
                if (insideTriangle(xSample, ySample, v.data()))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(xSample, ySample, v.data());
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    int sample_ind = eid + sp;
                    if (w_reciprocal > depth_sample_buf[sample_ind])
                    {
                        frame_sample_buf[sample_ind] = t.getColor();
                        depth_sample_buf[sample_ind] = w_reciprocal;
                    }
                }
            }
            // average ssaa samples
            set_pixel(Vector3f(x, y, 0), (frame_sample_buf[eid] + frame_sample_buf[eid + 1] +frame_sample_buf[eid + 2] +frame_sample_buf[eid + 3])/4.0f);

        }
}

void rst::rasterizer::rasterize_triangle_MSAA(const Triangle& t)
{
    auto v = t.toVector4();
    // calculate bounding box
    float minX,maxX,minY,maxY;
    minX = maxX = v[0].x();
    minY = maxY = v[0].y();
    for (int i = 1; i < 3; ++i)
    {
        minX = std::min(minX, v[i].x());
        minY = std::min(minY, v[i].y());
        maxX = std::max(maxX, v[i].x());
        maxY = std::max(maxY, v[i].y());
    }
    
    Vector2f offsets[4] = {{0.25f, 0.25f},{0.25f, 0.75f},{0.75f, 0.25f},{0.75f, 0.75f}}; 
    for (int y= std::floor(minY); y < std::ceil(maxY); y++)
        for (int x= std::floor(minX); x < std::ceil(maxX); x++)
        {
            int eid = get_index(x,y)* AAFactor;
            // MSAA
            int count_coverage = 0;
            int count_depth = 0;
            for (int sp=0; sp<4; ++sp)
            {
                float xSample = x + offsets[sp].x();
                float ySample = y + offsets[sp].y();
                if (insideTriangle(xSample, ySample, v.data()))
                {
                    count_coverage++;
                    auto[alpha, beta, gamma] = computeBarycentric2D(xSample, ySample, v.data());
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()); int sample_ind = eid + sp;
                    if (w_reciprocal > depth_sample_buf[sample_ind])
                    {
                        count_depth++;
                        depth_sample_buf[sample_ind] = w_reciprocal;
                    }
                }
            }
            // if touch the edge, do msaa average
            // the pixel is inside the triangle, not on the edge
            if (count_depth ==4)
            {
                set_pixel(Vector3f(x, y, 0), t.getColor());
            }
            // on the edge
            else if (count_depth >0)
            {
                Vector3f blendColor = count_coverage / 4.0f * t.getColor() + (1-count_coverage / 4.0f) * frame_buf[get_index(x,y)];
                set_pixel(Vector3f(x, y, 0), blendColor);
            }
        }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_sample_buf.begin(), frame_sample_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::lowest());
        std::fill(depth_sample_buf.begin(), depth_sample_buf.end(), std::numeric_limits<float>::lowest());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h );
    depth_buf.resize(w * h );
    frame_sample_buf.resize(w * h * AAFactor);
    depth_sample_buf.resize(w * h * AAFactor);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on