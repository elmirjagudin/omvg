#include <iostream>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common.h"
#include "synth.h"

using namespace cv;
using namespace std;

const Matx33d K = Matx33d(FX, 0, CX,
                          0, FY, CY,
                          0, 0,  1);

#define Z_DIST 20

#define IMG_WIN "synth points"

static void
set_point(Mat_<double> & frame, int point_num, double x, double y)
{
//    cout << x << ", " << y << endl;
    frame(0, point_num) = x;
    frame(1, point_num) = y;
}

void show_synth_points(Mat & points)
{
    Mat img(1000, 1000, CV_8UC3, Scalar(256, 256, 256));

    auto col = Scalar(0, 0, 0);
    for (int i = 0; i < points.rows; i += 1)
    {
        auto x = points.at<float>(i, 0);
        auto y = points.at<float>(i, 1);

        rectangle(img, Point(x-1, y-1), Point(x+1, y+1), col);
    }

    namedWindow(IMG_WIN, WINDOW_AUTOSIZE );
    imshow(IMG_WIN, img );
    waitKey(0);
//    destroyWindow(IMG_WIN);
}


static void
rt_vects(int i, Mat & rvec, Mat & tvec)
{
    auto yrot = ((float)i) / 4;
    auto t = ((float)i) / 3;

    rvec = (Mat_<double>(3,1) << 0, yrot, 0);
    tvec = (Mat_<double>(3,1) << t, 0, 0);

    cout << "i " << i << "tvec " << tvec << endl;
}

static void
synth(int i, Mat & imagePoints)
{
    vector<Point3f> points;

    for (int j = 0; j < 7; j += 3)
    {
        points.push_back(Point3f(0, 0, j+Z_DIST-10));
        points.push_back(Point3f(0, 5, j+Z_DIST-5));
        points.push_back(Point3f(5, 0, j+Z_DIST-5));
        points.push_back(Point3f(0, -5, j+Z_DIST-5));
        points.push_back(Point3f(-5, 0, j+Z_DIST-5));

        for (int i = -5; i <= 5; i += 1)
        {
            points.push_back(Point3f((float)i, 5, j+Z_DIST));
            points.push_back(Point3f((float)i, -5,j+Z_DIST));
        }

        for (int i = -4; i <= 4; i += 1)
        {
            points.push_back(Point3f(5, (float)i, j+Z_DIST));
            points.push_back(Point3f(-5, (float)i, j+Z_DIST));
        }
    }

    // for (int i = 0; i < points.size(); i += 1)
    // {
    //     auto p = points[i];
    //     cout << p.x << " " << p.y << " " << p.z << endl;
    // }

    // cout << "yrot " << yrot << " t " << t << endl;

    Mat rvec, tvec;
    rt_vects(i, rvec, tvec);

    Mat dist;

    // vector<Point3f> sliced_points;
    // for (int i = 0; i < 40; i += 1)
    // {
    //     sliced_points.push_back(points[i]);
    // }

    projectPoints(points, rvec, tvec, K, noArray(), imagePoints);
//    show_synth_points(imagePoints);
}

void
init_synth_points(vector<Mat> & points2d)
{
    for (int i = -1;
         i <= 1; //i <= 6;
         i += 2)
    {
        Mat imagePoints;

        // auto yr = ((float)i) / 4;
        // auto t = ((float)i) / 3;
        synth(i, imagePoints);


        Mat_<double> frame(2, imagePoints.rows);
        for (int j = 0; j < imagePoints.rows; j += 1)
        {
//            cout << imagePoints.at<float>(j, 0) << " " <<
//                imagePoints.at<float>(j, 1) << endl;
            set_point(frame, j,
                      imagePoints.at<float>(j, 0),
                      imagePoints.at<float>(j, 1));
        }

        points2d.push_back(Mat(frame));

//cout << imagePoints << endl;
    }
}

void
get_two_synth_views(openMVG::Mat & left, openMVG::Mat & right)
{
    vector<Mat> points2d;

    init_synth_points(points2d);

    auto pts = points2d[0];
    left = openMVG::Mat(2, pts.cols);
    for (int i = 0; i < pts.cols; i += 1)
    {
        left(0, i) = pts.at<double>(0, i);
        left(1, i) = pts.at<double>(1, i);
    }

    pts = points2d[1];
    right = openMVG::Mat(2, pts.cols);
    for (int i = 0; i < pts.cols; i += 1)
    {
        right(0, i) = pts.at<double>(0, i);
        right(1, i) = pts.at<double>(1, i);
    }

}
