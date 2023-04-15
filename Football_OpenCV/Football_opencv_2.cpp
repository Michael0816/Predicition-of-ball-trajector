#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv)
{
    // 打开视频文件
    VideoCapture cap("video.mp4");
    if (!cap.isOpened()) {
        std::cout << "Error opening video file" << std::endl;
        return -1;
    }

    // 创建窗口并调整其大小
    namedWindow("White Ball Detection", WINDOW_NORMAL);
    resizeWindow("White Ball Detection", 800, 600);

    while (true) {
        Mat frame;
        bool success = cap.read(frame);

        if (!success) {
            std::cout << "Error reading frame from video file" << std::endl;
            break;
        }

        // 对图像进行预处理，例如去除噪声，调整颜色平衡等等

        // 将图像转换为灰度图像
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // 应用阈值
        Mat thresh;
        threshold(gray, thresh, 200, 255, THRESH_BINARY);

        // 进行形态学运算，例如膨胀和腐蚀，以减少噪声并将球体区域增强
        Mat morph;
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(thresh, morph, MORPH_OPEN, kernel);

        // 在图像中查找轮廓
        std::vector<std::vector<Point>> contours;
        findContours(morph, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 遍历所有轮廓，并寻找满足条件的白色球
        for (const auto& contour : contours) {
            double area = contourArea(contour);

            // 检查轮廓是否是一个圆形
            if (contour.size() < 5) {
                continue;
            }
            std::vector<Point> approx;
            approxPolyDP(contour, approx, arcLength(contour, true) * 0.05, true);
            if (approx.size() != 1) {
                continue;
            }

            // 检查轮廓的颜色是否为白色
            Scalar color = mean(frame(contour));
            if (color[0] < 200 || color[1] < 200 || color[2] < 200) {
                continue;
            }

            // 绘制圆形轮廓和中心点
            Point2f center;
            float radius;
            minEnclosingCircle(contour, center, radius);
            circle(frame, center, radius, Scalar(0, 0, 255), 2);
            circle(frame, center, 2, Scalar(0, 255, 0), 3);

            // 计算球体的速度
            static Point2f prev_center(-1, -1);
            static double prev_time = 0;
            double curr_time = (double)cv::getTickCount() / cv::getTickFrequency();
            double time_diff = curr_time - prev_time;
            if (prev_center.x != -1 && prev_center.y != -1 && time_diff > 0) {
                double distance = norm(center - prev_center);
                double speed = distance / time_diff;
                std::cout << "Speed: " << speed << " pixels per second" << std::endl;
            }
            prev_center = center;
            prev_time = curr_time;
        }

        // 显示处理后的帧
        imshow("White Ball Detection", frame);

        // 按下ESC键退出循环
        if (waitKey(1) == 27) {
            break;
        }
    }

    // 释放视频捕获对象并关闭窗口
    cap.release();
    destroyAllWindows();

    return 0;
