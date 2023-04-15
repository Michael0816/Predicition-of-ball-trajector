#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv)
{
    // ����Ƶ�ļ�
    VideoCapture cap("video.mp4");
    if (!cap.isOpened()) {
        std::cout << "Error opening video file" << std::endl;
        return -1;
    }

    // �������ڲ��������С
    namedWindow("White Ball Detection", WINDOW_NORMAL);
    resizeWindow("White Ball Detection", 800, 600);

    while (true) {
        Mat frame;
        bool success = cap.read(frame);

        if (!success) {
            std::cout << "Error reading frame from video file" << std::endl;
            break;
        }

        // ��ͼ�����Ԥ��������ȥ��������������ɫƽ��ȵ�

        // ��ͼ��ת��Ϊ�Ҷ�ͼ��
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Ӧ����ֵ
        Mat thresh;
        threshold(gray, thresh, 200, 255, THRESH_BINARY);

        // ������̬ѧ���㣬�������ͺ͸�ʴ���Լ���������������������ǿ
        Mat morph;
        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(thresh, morph, MORPH_OPEN, kernel);

        // ��ͼ���в�������
        std::vector<std::vector<Point>> contours;
        findContours(morph, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // ����������������Ѱ�����������İ�ɫ��
        for (const auto& contour : contours) {
            double area = contourArea(contour);

            // ��������Ƿ���һ��Բ��
            if (contour.size() < 5) {
                continue;
            }
            std::vector<Point> approx;
            approxPolyDP(contour, approx, arcLength(contour, true) * 0.05, true);
            if (approx.size() != 1) {
                continue;
            }

            // �����������ɫ�Ƿ�Ϊ��ɫ
            Scalar color = mean(frame(contour));
            if (color[0] < 200 || color[1] < 200 || color[2] < 200) {
                continue;
            }

            // ����Բ�����������ĵ�
            Point2f center;
            float radius;
            minEnclosingCircle(contour, center, radius);
            circle(frame, center, radius, Scalar(0, 0, 255), 2);
            circle(frame, center, 2, Scalar(0, 255, 0), 3);

            // ����������ٶ�
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

        // ��ʾ������֡
        imshow("White Ball Detection", frame);

        // ����ESC���˳�ѭ��
        if (waitKey(1) == 27) {
            break;
        }
    }

    // �ͷ���Ƶ������󲢹رմ���
    cap.release();
    destroyAllWindows();

    return 0;
