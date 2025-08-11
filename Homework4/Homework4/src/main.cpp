#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

int fac(int n)
{
    if(n<=1)	return 1;
    return n*fac(n-1);
}

int comb(int n, int m)
{
    return fac(n) / fac(m) / fac(n - m); 
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    int n = points.size() - 1;

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        cv::Point2f point = {0,0};
        for (int i = 0; i <= n; i++)
        {
            point += comb(n, i) * std::pow(t, i) * std::pow(1-t, n-i) * points[i];
        }
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1)
        return control_points[0];
    
    std::vector<cv::Point2f> new_control_points;
    for (int i = 0; i < control_points.size()-1; i++)
    {
        cv::Point2f p0 = control_points[i];
        cv::Point2f p1 = control_points[i+1];
        cv::Point2f p2 = p0 * (1-t) + p1 * t;
        new_control_points.push_back(p2);
    }
    return recursive_bezier(new_control_points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}
int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }
        if (key == 13)//Enter key
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
