#include <iostream>
#include <librealsense2/rs.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include <iostream>
#include "opencv2/imgproc/types_c.h"
//#include <librealsense/wrappers/opencv/cv-helpers.hpp>
//#include "../include/librealsense/wrappers/opencv/cv-helpers.hpp"
#include <sstream>
#include <fstream>
#include "ros/ros.h"
#include "../msg/Vector3.h"
#include "../include/quanternionMath/quanternionMath.h"

using namespace std;
using namespace cv;

cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}


rs2::pipeline_profile profile;

enum class direction //class needed for camera calibration, provided by Intel
{
    to_depth,
    to_color
};
double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) // class used to compute angles in order to pick out the sqauares in an image
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

rs2::frameset getAlignedFramset() // color images and positional data are sent sepretely from the realsense camera and need to be aligned in order to be used
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    // cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_COLOR);

    profile = pipe.start(cfg);

    for (auto i = 0; i < 30; ++i)
        pipe.wait_for_frames();
    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    direction dir = direction::to_depth; // Alignment direction
    rs2::frameset frameset = pipe.wait_for_frames();
    // ...
    frameset = align_to_depth.process(frameset);
    if (dir == direction::to_depth)
    {
        // Align all frames to depth viewport
        frameset = align_to_depth.process(frameset);
    }
    else
    {
        // Align all frames to color viewport
        frameset = align_to_color.process(frameset);
    }

    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();

    // int widthColor = color.get_width();
    // int widthdepth = depth.get_width();
    // int heightColor = color.get_height();
    // int heightDepth = depth.get_height();

    // cout << widthColor << endl;
    // cout << widthdepth << endl;
    // cout << heightColor << endl;
    // cout << heightDepth << endl;
    return frameset;
}
rs2_intrinsics getCameraIntrinsics() // camera intrincis are needed to account for the distortion produced by the camera lens and alignement of the images. 
{
    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto intr = stream.get_intrinsics(); // Calibration data

    return intr;
}
vector<vector<Point>> find_squares(Mat &image, vector<vector<Point>> &squares) // this method is for finding a square piece of paper in an image.
{
    // blur will enhance edge detection
    Mat blurred(image);
    medianBlur(image, blurred, 5);
    imshow("bllurred image", blurred);
    waitKey(0);

    Mat gray0(blurred.size(), CV_8U), gray;
    vector<vector<Point>> contours;

    // find squares in every color plane of the image
    for (int c = 0; c < 3; c++)
    {
        int ch[] = {c, 0};
        mixChannels(&blurred, 1, &gray0, 1, ch, 1);

        // try several threshold levels
        const int threshold_level = 2;
        for (int l = 0; l < threshold_level; l++)
        {
            // Use Canny instead of zero threshold level!
            // Canny helps to catch squares with gradient shading
            if (l == 0)
            {
                Canny(gray0, gray, 10, 20, 3); //

                // Dilate helps to remove potential holes between edge segments
                dilate(gray, gray, Mat(), Point(-1, -1));
            }
            else
            {
                gray = gray0 >= (l + 1) * 255 / threshold_level;
            }

            // Find contours and store them in a list
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            // Test contours
            vector<Point> approx;
            for (size_t i = 0; i < contours.size(); i++)
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.05, true);

                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if (approx.size() == 4 &&
                    fabs(contourArea(Mat(approx))) > 1000 && fabs(contourArea(Mat(approx))) < 100000 &&
                    isContourConvex(Mat(approx)))
                {
                    double maxCosine = 0;

                    for (int j = 2; j < 5; j++)
                    {
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    if (maxCosine < .3)
                        squares.push_back(approx);
                }
            }
        }
    }
    cout << "done finding squares" << endl;
    return squares;
}
cv::Mat debugSquares(std::vector<std::vector<cv::Point>> squares, cv::Mat image)// this method is only to view the squares found in the workspace, just for debugging
{

    for (int i = 0; i < squares.size(); i++)
    {
        // draw contour
        cv::drawContours(image, squares, i, cv::Scalar(255, 0, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

        // // draw bounding rect
        // cv::Rect rect = boundingRect(cv::Mat(squares[i]));
        // cv::rectangle(image, rect.tl(), rect.br(), cv::Scalar(0, 255, 0), 2, 8, 0);

        // // draw rotated rect
        // cv::RotatedRect minRect = minAreaRect(cv::Mat(squares[i]));
        // cv::Point2f rect_points[4];
        // minRect.points(rect_points);
        // for (int j = 0; j < 4; j++)
        // {
        //     cv::line(image, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 0, 255), 1, 8); // blue
        // }
    }
    Point center;
    for (int i = 0; i < 4; i++)
    {
        string label = to_string(squares.at(0).at(i).x) + ", " + to_string(squares.at(0).at(i).y);
        center = squares.at(0).at(i);
        putText(image, label, center, FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
        circle(image, center, 1, CV_RGB(255, 0, 0), 3);
    }

    imshow("squares Found", image);
    waitKey(0);
    return image;
}
void savePublishedData(geometry_msgs::Vector3 paperVector) // This method is to save the point found in a text file so that you know all the points that were sent over the ROS topic.
{
    string filename("publishedPoints.txt");
    ofstream file_out;

    file_out.open(filename, std::ios_base::app);
    file_out << paperVector.x << " " << paperVector.y << " " << paperVector.z << endl;
    cout << "added point to txt file" << endl;
}
void PublishPointData() // this method is to publish the data to the ROS topic, yet to be completed.
{

    return;
}

int main(int argc, char **argv)
{

    //------------------Start ROS node To Publish Data-------------------------------------
    ros::init(argc, argv, "PaperVector");
    ros::NodeHandle n;

    //------------------Start ROS Publisher-------------------------------------
    ros::Publisher vector_pub = n.advertise<geometry_msgs::Vector3>("PaperVector", 1000);

    bool PublishAgain = true;
    do
    {
        //------------------Camera Setup-------------------------------------------------------------------
        rs2::frameset frameset = getAlignedFramset();
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        const rs2_intrinsics intrin = getCameraIntrinsics();

        //------------------image Setup-------------------------------------------------------------------
        auto image = frame_to_mat(color);

        // cv::imshow("image ", image); // uncommet to show image
        // imwrite("image", image);     // uncomment to save image

        //------------------find paper as a square in the image------------------------------------------
        vector<vector<Point>> squares;
        squares = find_squares(image, squares);
        debugSquares(squares, image);
        float paperPixelCord[2];
        paperPixelCord[0] = squares.at(0).at(0).x;  // the first square foud will always be the largest, so if incorrect sqaures are found as long as they are smaller then the paper, they will be excluded.
        paperPixelCord[1] = squares.at(0).at(0).y;

        QMath::printArray(paperPixelCord, 2, "papper pixel cords");

        //------------------get depth of pixel------------------------------------------------------------------------
        float pixelDepth = depth.get_distance(paperPixelCord[0], paperPixelCord[1]);

        //------------------get real world cordinates of pixel using projection------------------------------------------
        float pixelProjected[3];
        rs2_deproject_pixel_to_point(pixelProjected, &intrin, paperPixelCord, pixelDepth);
        QMath::printArray(pixelProjected, 3, "pixel projected");

        //------------------Quanternion Math to get transformed vector-----------------------------------------
        // float pointArray[3] = {0.148412, -0.167296, 0.955};                                                         // testing point [0.148412 -0.167296 0.955] robot cords should be .163, -.414, .06
        float translation[3] = {-0.3634422254733067, -0.6520691528769079, 0.8675182456610416};                      //  format = {x y z}
        float quanternion[4] = {0.19093701021630496, -0.8941473997722545, 0.39227689972072916, -0.100808330785048}; // format = {qw rx ry rz}
        vector<float> transformedPointVector;
        transformedPointVector = QMath::transformPoint(pixelProjected, translation, quanternion); // function to transform the point from camera space to robot base cordinates.

        //------------------Publish Pixel cords and save to file-----------------------------------------
        geometry_msgs::Vector3 paperVector;
        paperVector.x = transformedPointVector.at(0);
        paperVector.y = transformedPointVector.at(1);
        paperVector.z = transformedPointVector.at(2);

        cout << paperVector.x << " transformed paper vector x " << endl;
        cout << paperVector.y << " transformed paper vector y " << endl;
        cout << paperVector.z << " transformed paper vector z " << endl;

        vector_pub.publish(paperVector);

        savePublishedData(paperVector);

        //------------------check to see if user wants to publish again-----------------------------------------
        char x;
        cout << "type y to publish again and n to stop" << endl;
        cin >> x;
        if (x == 'Y' || x == 'y')
        {
            PublishAgain = true;
        }
        else
        {
            PublishAgain = false;
        }

    } while (PublishAgain);

    return 1;
}