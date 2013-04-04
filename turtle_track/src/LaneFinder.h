//Based on the work of Irene Markelic
//original code at: http://sourceforge.net/projects/linefinder/

#ifndef LANEFINDER_H
#define	LANEFINDER_H
#include <stdarg.h>
#include <stdlib.h>
#include <cv.h>                 /* Contains all cv* function calls */
#include <highgui.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <math.h>
#include <fstream>
#include <algorithm>
#include <vector>
#include <list>
#include <queue>
#include <stdio.h>
#include <iomanip> 

void displayImage(IplImage* ImageToDisplay, char* NameOfWindow);

/**
 * The struct housing line pixels
 */
struct line_points
{
		  CvPoint point;
		  float orientation;
		  bool endpoint;
		  bool startpoint;
};

class LaneFinder {
private:
    //width and height of processed image
    int WIDTH;
    int HEIGHT;

public:
    CvMemStorage *STORAGE;
    
    //constructor, given width and height of image to be processed
    LaneFinder(int width, int height);
    
    //destructor
    ~LaneFinder();
    
    int getWidth();
    int getHeight();
    
    void getHighestYIndex(std::vector<struct line_points*> &longestLine, int &highest_y_index, CvPoint &yPoint);
    
    /**
     * converts pixels from the extracted lane from the lineStruct to CvPoints
     */
    void convertLineStructToVector(std::vector< CvPoint > &vec, std::vector<struct line_points*> &lp);
    
    void copy_line_points_Struct(line_points **p_target, line_points* p_src);
    
    void doJoiningTwoOrient(std::vector< std::vector< struct line_points* > > &bin,
        std::vector< struct line_points* > &longestLine, int dist);
    
    void findLines(IplImage* img, double* alpha, int &x_pos, std::vector< struct line_points*> &longestLine,
        int lowerLimit, int higherLimit, int yLimit, std::vector< std::vector< struct line_points* > > &bin,
        std::vector< std::vector< struct line_points* > > &binRight, int minSize, int lengthTangent );
    
    /**
     * Takes a images, applies the Canny filter and creates line segments that start
     * left and go right
     * @param img: input image
     * @return vector of line segments that start left and go right
     */
    std::vector< std::vector< struct line_points* > > binLinesLR(IplImage* img);
    
     /**
     * Takes a images, applies the Canny filter and creates line segments that start
     * right and go left
     * @param img: input image
     * @return vector of line segments that start right and go left
     */
    std::vector< std::vector< struct line_points* > > binLinesRL(IplImage* img);
    
    void getTangentParams(std::vector< struct line_points*> &longestLine, int &highest_y_index, double &m,
        double &b, double &angle, CvPoint &startPoint, CvPoint &endPoint, int length, int imgWidth, IplImage *img);
    
    /*Extend the detected line to the bottom of the image using the tangent to the line
     */
    void doExtrapolation(std::vector< struct line_points*> &longestLine, IplImage* img, double m, double b, CvPoint &point_y, int tangentLength);
    
    /**
     * calculates pixel orientations of image img
     */
    IplImage* getOrientationImage(IplImage *img);
    
    /*
     * Given an image, a point and orientation data for that image, looks recursively for
     * neighbours of that point along lines going left to right
     */
    void getNeighboursOrderedLR(IplImage* img, int x, int y,  std::vector<struct line_points*> &points,
        IplImage* orientationImg, int orientation);
    
    /*
     * Given an image, a point and orientation data for that image, looks recursively for
     * neighbours of that point along lines going right to left
     */
    void getNeighboursOrderedRL(IplImage* img, int x, int y,  std::vector<struct line_points*> &points,
        IplImage* orientationImg, int orientation);
    
    void printLane(IplImage* imgColor, const std::vector< struct line_points* > &longestLine);
    
     /**
      *deletes the found line data structure to free memory. 
      */
    void deleteLine( std::vector< struct line_points*> const &longestLine );
    
    /**
     * Extracts the right lane from a given image, along with other data about the lane
     * @param imgInput: the image where we look for the lane
     * @return imgOutput: the original image with the detected lane highlighted
     * @return lineRight: the detected line as a vector of CvPoints
     * @param searchLimitLeft: defines boundary at image bottom where we look for the line
     * @param searchLimitRight: defines boundary at image bottom where we look for the line
     * @param minSize: we are looking for a line with this minimal length in pixels
     * @param lengthTangent: used to extrapolate line to bottom of the image
     */
    void extractLane(IplImage *imgInput, IplImage *imgOutput, std::vector<CvPoint> &lineRight, 
        int searchLimitLeft, int searchLimitRight, int minSize, int lengthTangent, double &alpha);

    /**
     * A scalar Kalman filter, for the interval at the bottom of the image where the lane is thought to be
     * @param state_mean_prev: mean of the previous estimate
     * @param state_var_prev: variance of the previous estimate
     * @param measurement: the measurement
     * @param state_mean_estimate: mean of the calculated estimate
     * @param state_var_estimate: variance of the calculated estimate
     */
    void doKalmanFiltering(double state_mean_prev, double state_var_prev, double measurement,
        double &state_mean_estimate, double &state_var_estimate);
};

class Tangent {
private:
    int WIDTH;
    int HEIGHT;
public:
    Tangent(int width, int height);
    ~Tangent();
    int getWidth();
    int getHeight();
    
    void calTangentParams(CvPoint &p_1, CvPoint &p_2, double &m, double &b);
    
    static void calTangentParamsInv(CvPoint &p_1, CvPoint &p_2, double &m, double &b);
    
    static double calTangentValue(double m, int x, double b);
};

#endif	/* LANEFINDER_H */

