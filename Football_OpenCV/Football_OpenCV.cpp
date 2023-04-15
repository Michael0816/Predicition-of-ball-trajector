#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    // 打开视频文件
    VideoCapture cap("video.mp4");

    // 检查视频是否打开
    if (!cap.isOpened()) {
        cout << "Error opening video file" << endl;
        return -1;
    }

    // 定义变量
    Mat frame, hsv, mask, eroded, dilated;
    int minRadius = 5, maxRadius = 20;
    vector<Vec3f> circles;
    Point prev_center(0, 0);
    double prev_time = 0.0, current_time = 0.0, dt = 0.0, speed = 0.0;

    // 读取视频帧直到结束
    while (cap.read(frame)) {
        // 将帧转换为HSV颜色空间
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // 创建白色颜色的掩码
        inRange(hsv, Scalar(0, 0, 200), Scalar(180, 40, 255), mask);

        // 腐蚀和膨胀掩码以去除噪声
        erode(mask, eroded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        dilate(eroded, dilated, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        // 在掩码中查找圆形
        HoughCircles(dilated, circles, HOUGH_GRADIENT, 1, 20, 100, 30, minRadius, maxRadius);

        // 如果找到圆形，则绘制并计算速度
        if (circles.size() > 0) {
            // 获取圆形的中心和半径
            Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
            int radius = cvRound(circles[0][2]);

            // 绘制圆形中心
            circle(frame, center, 3, Scalar(0, 255, 0), -1, LINE_AA);

            // 绘制圆形轮廓
            circle(frame, center, radius, Scalar(0, 0, 255), 3, LINE_AA);

            // 计算速度
            prev_time = current_time;
            current_time = (double)getTickCount() / getTickFrequency();
            dt = current_time - prev_time;
            speed = norm(center - prev_center) / dt;
            prev_center = center;
        }

        // 显示视频帧
        imshow("Video", frame);

        // 按ESC键退出循环
        if (waitKey(1) == 27) {
            break;
        }
    }

    // 释放视频捕获和窗口
    cap.release();
    destroyAllWindows();

    return 0;
}
