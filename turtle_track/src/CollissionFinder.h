#ifndef COLLISSIONFINDER_H
#define	COLLISSIONFINDER_H
#define ROBOT_WIDTH 95
#define LINE_WIDTH 16
#include <cv.h>
#include <stdlib.h>

using namespace cv;

//fill the black corners of the IPM image, avoid false collissions
void fillRoadColor(Mat& rgbWorFrame, Vec3b roadColor);
Vec3b extractRoadColor(Mat img);

//check if a pixel is collission or not, return 255/0
uchar isCollisionBin(Vec3b pixel, Vec3b color, int threshold);

//check if a pixel is a collission or not, return 255/0 for each channel
Vec3b isCollision(Vec3b pixel, Vec3b color, int threshold);

//get a binary (for each channel) collision image
void findCollisions(Mat img, Mat &binaryImg, Vec3b color, int threshold);

//get a binary collision image
void findCollisionsBin(Mat img, Mat &binaryImg, Vec3b color, int threshold);

//check the interval along the line through which the robot passes for collisions, based on 3chan collission image
int avoidCollision(Mat colWorImg, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo);

//check the interval along the line through which the robot passes for collisions, based on 1chan collission image
int avoidCollisionBin(Mat colWorImg, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo);

////check the interval along the line through which the robot passes for collisions, based on 1chan collission image
////update road color to avoid lighting differences
int avoidCollisionBinUpdate(Mat colWorImg, Mat rgbWorFrame, Vec3b& roadColor, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo);
#endif
