#include <chrono>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> points = control_points;
    while (points.size() > 1) {
        std::vector<cv::Point2f> temp;
        for (int i = 0; i < points.size() - 1; i++) {
            const float x = (1 - t) * points[i].x + t * points[i + 1].x;
            const float y = (1 - t) * points[i].y + t * points[i + 1].y;
            temp.emplace_back(x, y);
        }
        points = std::move(temp);
    }
    return points[0];
}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window, cv::Mat accum_buffer, bool reset_buffer = false) {
    if (control_points.size() < 2) return;

    if (accum_buffer.empty() || reset_buffer) {
        window.convertTo(accum_buffer, CV_32FC3, 1.0 / 255.0); 
    }

    const float step = 1.0f / window.cols;
    const cv::Vec3f color(0.0f, 1.0f, 0.0f);

    for (float t = 0.0f; t <= 1.0f; t += step) {
        auto point = recursive_bezier(control_points, t);

        int fx = static_cast<int>(point.x);
        int fy = static_cast<int>(point.y);
        float dx = point.x - fx;
        float dy = point.y - fy;

        if (fx >= 0 && fx < window.cols - 1 && fy >= 0 && fy < window.rows - 1) {
            cv::Vec3f& pA = accum_buffer.at<cv::Vec3f>(fy, fx);
            cv::Vec3f& pB = accum_buffer.at<cv::Vec3f>(fy, fx + 1);
            cv::Vec3f& pC = accum_buffer.at<cv::Vec3f>(fy + 1, fx);
            cv::Vec3f& pD = accum_buffer.at<cv::Vec3f>(fy + 1, fx + 1);

            pA += (1 - dx) * (1 - dy) * color;
            pB += dx * (1 - dy) * color;
            pC += (1 - dx) * dy * color;
            pD += dx * dy * color;
        }
    }
    accum_buffer.convertTo(window, CV_8UC3, 255.0f);
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::Mat accum_buffer;//¸¡µã»º³åÇø

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window, accum_buffer);

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
