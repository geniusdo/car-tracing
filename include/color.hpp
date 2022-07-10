// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
namespace color
{
    enum color_kind
    {
        blue,
        white,
        gray,
        purple,
        yellow,
        green,
        red,
        black,
        cyan,
        orange,
        test
    };
    void color_Range(cv::InputArray img, cv::OutputArray imgThresholded, const color_kind &ctrl)
    {
        cv::Mat imghsv;
        cv::cvtColor(img, imghsv, cv::COLOR_BGR2HSV);
        switch (ctrl)
        {
        case blue:
        {
            cv::inRange(imghsv, cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255), imgThresholded); //蓝色
            break;
        }
        case white:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 221), cv::Scalar(180, 30, 255), imgThresholded); //白色
            break;
        }
        case gray:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 46), cv::Scalar(180, 43, 220), imgThresholded); //灰色
            break;
        }
        case purple:
        {
            cv::inRange(imghsv, cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255), imgThresholded); //紫色
            break;
        }
        case yellow:
        {
            cv::inRange(imghsv, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), imgThresholded); //黄色
            break;
        }
        case green:
        {
            cv::inRange(imghsv, cv::Scalar(40, 43, 110), cv::Scalar(77, 150, 255), imgThresholded); //绿色
            break;
        }
        case red:
        {
            // Mat imgThresholded1, imgThresholded2;
            inRange(imghsv, cv::Scalar(0, 150, 150), cv::Scalar(30, 255, 255), imgThresholded); //红色
            // add(imgThresholded1, imgThresholded2, imgThresholded, Mat(), CV_64FC1);
            // cv::inRange(imghsv, cv::Scalar(0, 0, 255), cv::Scalar(90, 250, 255), imgThresholded);
            break;
        }
        case black:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 90), imgThresholded); //黑色
            break;
        }
        case cyan:
        {
            cv::inRange(imghsv, cv::Scalar(78, 43, 46), cv::Scalar(99, 255, 255), imgThresholded); //青色
            break;
        }
        case orange:
        {
            cv::inRange(imghsv, cv::Scalar(11, 43, 46), cv::Scalar(25, 255, 255), imgThresholded); //橙色
            break;
        }
        case test:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 0), cv::Scalar(40, 255, 255), imgThresholded); // test
            break;
        }
        default:
        {
            cv::inRange(imghsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255), imgThresholded);
            break;
        }
        }
    }
    bool color_center(cv::InputArray img_two, 
                      cv::Point2f &photo_center)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarcy;
        findContours(img_two, contours, hierarcy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty())
        {
            return false;
        }
        else
        {
            std::vector<cv::Point> contour;
            unsigned long max_contour = 0;
            double max_area = cv::contourArea(contours[0]);
            for (unsigned long i = 1; i < contours.size(); i++)
            {
                double temp_area = cv::contourArea(contours[i]);
                if (max_area < temp_area)
                {
                    max_area = temp_area;
                    max_contour = i;
                }
            }
            contour = contours[max_contour];
            if (max_area < 50.0)
            {
                photo_center.x = 0;
                photo_center.y = 0;
                return false;
            }
            else
            {
                std::vector<cv::RotatedRect> box(contour.size()); //最小外接矩形
                cv::Point2f rect[4];

                box[0] = cv::minAreaRect(cv::Mat(contour));
                box[0].points(rect); //最小外接矩形的4个端点
                int x = 0, y = 0;
                for (int j = 0; j < 4; j++)
                {
                    x = x + rect[j].x;
                    y = y + rect[j].y;
                }
                photo_center.x = x / 4;
                photo_center.y = y / 4;
                return true;
            }
        }
    }
    bool isstopped(double time,double limvel, const cv::Point2f &pre_cen, const cv::Point2f &cen,double coff)
    {
        double x_sub = (pre_cen.x - cen.x)*coff;
        double y_sub = (pre_cen.y - cen.y)*coff;
        double diatance = sqrt(pow(x_sub, 2) + pow(y_sub, 2));
        if (diatance < limvel * time)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}