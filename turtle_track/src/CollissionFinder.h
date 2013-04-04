#ifndef COLLISSIONFINDER_H
#define	COLLISSIONFINDER_H
#define ROBOT_WIDTH 95
#define LINE_WIDTH 16
#include <cv.h>
#include <stdlib.h>

using namespace cv;

void fillRoadColor(Mat& rgbWorFrame, Vec3b roadColor);
Vec3b extractRoadColor(Mat img);
uchar isCollisionBin(Vec3b pixel, Vec3b color, int threshold);
Vec3b isCollision(Vec3b pixel, Vec3b color, int threshold);
void findCollisions(Mat img, Mat &binaryImg, Vec3b color, int threshold);
void findCollisionsBin(Mat img, Mat &binaryImg, Vec3b color, int threshold);
int avoidCollision(Mat colWorImg, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo);
int avoidCollisionBin(Mat colWorImg, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo);
int avoidCollisionBinUpdate(Mat colWorImg, Mat rgbWorFrame, Vec3b& roadColor, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo);
#endif
