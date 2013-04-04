//Based on the work of Irene Markelic
//original code at: http://sourceforge.net/projects/linefinder/

#include "LaneFinder.h"

using namespace std;

/** Compute the angle between two intersecting lines. The lines are
 * given by 2 points through which they pass
 **/
double angleBetweenLines(CvPoint start_line1, CvPoint end_line1, CvPoint start_line2, CvPoint end_line2) {
    int x1=end_line1.x - start_line1.x;
    int y1=end_line1.y - start_line1.y;
    
    int x2=end_line2.x - start_line2.x;
    int y2=end_line2.y - start_line2.y;
    
    //check if either of the lines has matching endpoint, being not defined properly
    if ((x1 == 0 && y1 ==0) || (x2 == 0 && y2 == 0)) return 0.1;
    
    double length_line1 = sqrt(x1*x1 + y1*y1);
    double length_line2 = sqrt(x2*x2 + y2*y2);
    double dot_prod = x1*x2 + y1*y2;
    
    double cos_alpha = dot_prod / (length_line1 * length_line2);
    double alpha = acos(cos_alpha);
    
    return alpha;
}

/**
 * Compute the distance between two pixels
 */
double distBetweenPoints(CvPoint const &p1, CvPoint const &p2) {
    int x_dist = p1.x - p2.x;
    int y_dist = p1.y - p2.y;
    
    double dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    
    return dist;
}

/**
 * Display image in a window until the next keystroke
 */
void displayImage(IplImage* image, char *windowName) {
    cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cvShowImage(windowName, image);
    cvWaitKey(0);
    cvDestroyWindow(windowName);
}

bool lineSort(struct line_points* a, struct line_points* b) {
	return (a->point.y > b->point.y);
}

/**
 * LaneFinder constructor, given width and height of the image in which we look for a line.
 */
LaneFinder:: LaneFinder(int width, int height) {
    WIDTH = width;
    HEIGHT = height;
    STORAGE = cvCreateMemStorage(0);
}

/**
 * LaneFinder destructor, clears up the storage from memory
 */
LaneFinder:: ~LaneFinder() {
    if (STORAGE !=NULL) {
        cvClearMemStorage(STORAGE);
        cvReleaseMemStorage(&STORAGE);
        STORAGE = NULL;
    }
}

int LaneFinder::getWidth() {
    return WIDTH;
}

int LaneFinder::getHeight() {
    return HEIGHT;
}

void LaneFinder::getHighestYIndex(std::vector<struct line_points*> &longestLine, int &highest_y_index, CvPoint &yPoint) {
    int y_curr, y_highest;
    y_highest = 0;
    
    for(int i = 0; i <  longestLine.size(); i++) {
        y_curr = longestLine[i]->point.y;
        if (y_curr > y_highest) {
            y_highest = y_curr;
            highest_y_index = i;
            yPoint = longestLine[i]->point;
        }
    }
}

/**
 * converts pixels from the extracted lane from the lineStruct to CvPoints
 */
void LaneFinder::convertLineStructToVector(std::vector< CvPoint > &vec, std::vector<struct line_points*> &lp)
{
    for(int i = 0; i< lp.size(); i++) {
        CvPoint p=cvPoint(lp[i]->point.x,lp[i]->point.y);
        vec.push_back(p);
    }
}

void LaneFinder::findLines(IplImage *img, double* alpha,int &x_pos, std::vector< struct line_points*> &longestLine,
        int lowerLimit, int higherLimit, int yLimit, std::vector< std::vector< struct line_points* > > &binLR, 
        std::vector< std::vector< struct line_points* > > &binRL, int minSize, int lengthTangent) {
    cvClearMemStorage(STORAGE);
    
    static double alpha_prev = 0;
    static int x_pos_prev = 0;
    
    if ((binLR.size() <= 0) || (binRL.size() <= 0)) {
        *alpha = alpha_prev;
        x_pos = x_pos_prev;
        return;
    }
    
    //find longest line in binLR within the area of interest
    int maxLength = -1;
    int indexMaxLR = -1;
    int n = binLR.size();
    for(int i=0; i < n;i++) {
        int m = binLR[i].size();
        if(m > maxLength && (binLR[i][0]->point.x<=higherLimit) && (binLR[i][0]->point.x>=lowerLimit) &&  binLR[i][0]->point.y > yLimit) {
            maxLength = m;
            indexMaxLR = i;
        }
    }
    
    //find longest line in binRL within the area of interest
    maxLength = -1;
    int indexMaxRL = -1;
    n = binRL.size();
    for(int i=0; i < n;i++) {
        int m = binRL[i].size();
        if(m > maxLength && (binRL[i][0]->point.x<=higherLimit) && (binRL[i][0]->point.x>=lowerLimit) &&  binRL[i][0]->point.y > yLimit) {
            maxLength = m;
            indexMaxRL = i;
        }
    }
    
    int l_r = -1;
		  if(indexMaxLR>=0 || indexMaxRL>=0)
		  {
					 if(indexMaxLR>=0 && indexMaxRL>=0)
					 {
								if(binLR[indexMaxLR].size()>=binRL[indexMaxRL].size())
								{

										  //copy into longestLine
										  for(int i=0; i< binLR[indexMaxLR].size(); i++)
										  {
													 struct line_points *p;
													 copy_line_points_Struct(&p, binLR[indexMaxLR][i]);
													 longestLine.push_back(p);
										  }
										  l_r=1;//next check binRL
								}
								else
								{
										  //copy into longestLine
										  for(int i=0; i< binRL[indexMaxRL].size(); i++)
										  {
													 struct line_points *p;
													 copy_line_points_Struct(&p, binRL[indexMaxRL][i]);
													 longestLine.push_back(p);
										  }
										  l_r=2;//next check binLR
								}
					 }
					 else if(indexMaxLR >=0 && indexMaxRL<0)
					 {
								//copy into longestLine
								for(int i=0; i< binLR[indexMaxLR].size(); i++)
								{
										  struct line_points *p;
										  copy_line_points_Struct(&p, binLR[indexMaxLR][i]);
										  longestLine.push_back(p);
								}
								l_r=1;//next check binRL

					 }
					 else if(indexMaxRL >=0 && indexMaxLR <0)
					 {
								for(int i=0; i< binRL[indexMaxRL].size(); i++)
								{
										  struct line_points *p;
										  copy_line_points_Struct(&p, binRL[indexMaxRL][i]);
										  longestLine.push_back(p);
								}
								l_r=2;//next check binLR

					 }
        
        bool notDone = true;
        int diff = 40;
        while(notDone) {
            notDone = false;
            int binSize = longestLine.size();
            if(l_r % 2 == 0) { //check binLR
                doJoiningTwoOrient(binLR, longestLine, diff);
                l_r++;
            } else { //check binRL
                doJoiningTwoOrient(binRL, longestLine, diff);
                l_r++;
            }
            
            diff += 10;
            if (diff >= 100) {break;}
            notDone = (binSize < longestLine.size());
        }
    }
    
    if(longestLine.size() > minSize) {
        int highest_y_index;
        double m, b, angle;
        CvPoint startPoint,  endPoint;
        getTangentParams(longestLine, highest_y_index, m, b, angle, startPoint, endPoint, lengthTangent, img->width, img);        
        if (startPoint.y < img->height - 5 && startPoint.x < img->width) {
            doExtrapolation(longestLine, img, m, b, startPoint, lengthTangent);
        }
				sort(longestLine.begin(), longestLine.end(), lineSort);
        getTangentParams(longestLine, highest_y_index, m, b, angle, startPoint, endPoint, lengthTangent, img->width, img);
        *alpha = angle;
        x_pos = startPoint.x;
        alpha_prev = angle;
        x_pos_prev = x_pos;
    } else {
        //clean up memory
        for(int i = 0; i < longestLine.size(); i++) longestLine.pop_back();
        longestLine.clear();
        *alpha = -1;
        x_pos = -1;
    }
}

/**
 * Takes a images, applies the Canny filter and creates line segments that start
 * left and go right
 * @param img: input image
 * @return vector of line segments that start left and go right
 */
vector< vector< struct line_points* > > LaneFinder::binLinesLR(IplImage* img) {
    vector< vector< struct line_points* > > binLR;
    IplImage* canny_img = cvCloneImage(img);
    cvClearMemStorage(STORAGE);
    
    //get the orientation data for each pixel
    static IplImage* orient = getOrientationImage(img);
    
    //apply the Canny filter that detects (based on parameters) edges in 
    //the image.canny_img will be the edge only version of img
    cvCanny(img, canny_img, 2000, 2300, 5);
    
    //in canny_img we will look for long edges going either left to right or 
    //the other way around. in the initial canny_img edge pixels have value 255,
    //while the others have value 0. we start from the bottom line and starting
    //from a pixel we try to find the longest line containing that pixel, using
    //the orientation data. we mark pixels that have already been detected as part
    //of an edge by setting them to 70.
    int pix_canny;
    for(int b = img->height - 1; b > 1; b--) {
        for(int a = 0; a < img->width; a++) {
            //take each pixel in the canny image
            pix_canny = (int) (CV_IMAGE_ELEM(canny_img, uchar, b, a));
            
            //if there is a line at that pixel
            if (pix_canny == 255) {
                struct line_points *p;
                vector<struct line_points*> points;
                
                //mark pixel as read
                CV_IMAGE_ELEM(canny_img, uchar, b, a) = (uchar) 70;
                
                p = (struct line_points *) malloc (sizeof(struct line_points));
                if (p == NULL) exit(-1);
                
                p->point = cvPoint(a,b);
                float currentOrientation = (float) CV_IMAGE_ELEM(orient, float, b, a);
                p->orientation = currentOrientation;
                p->endpoint = false;
                p->startpoint = true;
                points.push_back(p);
                getNeighboursOrderedLR(canny_img, a, b, points, orient, (int)currentOrientation);
                //relevant line
		//TODO
                if (points.size() >= 5) {
                    binLR.push_back(points);
                }
                //noise
                else {
                    for(int c = 0; c < points.size(); c++) {
                        //make these pixels black in the image
                        CV_IMAGE_ELEM(canny_img, uchar, points[c]->point.y, points[c]->point.x) = (uchar)0;
                        if (points[c] != NULL) {
                            free(points[c]);
                            points[c] = NULL;
                        }
                    }
                    points.clear();
                }
            }
        }
    }
    return binLR;
}


/**
 * Takes a images, applies the Canny filter and creates line segments that start
 * right and go left
 * @param img: input image
 * @return vector of line segments that start right and go left
 */
vector< vector< struct line_points* > > LaneFinder::binLinesRL(IplImage* img) {
    vector< vector< struct line_points* > > binRL;
    IplImage* canny_img = cvCloneImage(img);
    cvClearMemStorage(STORAGE);
    
    static IplImage* orient = getOrientationImage(img);
    cvCanny(img, canny_img, 2000, 2300, 5);
    
    int pix_canny;
    for(int b = img->height - 1; b > 1; b--) {
        for(int a = 0; a < img->width; a++) {
            //take each pixel in the canny image
            pix_canny = (int) (CV_IMAGE_ELEM(canny_img, uchar, b, a));
            
            //if there is a line at that pixel
            if (pix_canny == 255) {
                struct line_points *p;
                vector<struct line_points*> points;
                
                //mark pixel as read
                CV_IMAGE_ELEM(canny_img, uchar, b, a) = (uchar) 70;
                
                p = (struct line_points *) malloc (sizeof(struct line_points));
                if (p == NULL) exit(-1);
                
                p->point = cvPoint(a,b);
                float currentOrientation = (float) CV_IMAGE_ELEM(orient, float, b, a);
                p->orientation = currentOrientation;
                p->endpoint = false;
                p->startpoint = true;
                points.push_back(p);
                getNeighboursOrderedRL(canny_img, a, b, points, orient, (int)currentOrientation);
                //relevant line
		//TODO
                if (points.size() >= 5) {
                    binRL.push_back(points);
                }
                //noise
                else {
                    for(int c = 0; c < points.size(); c++) {
                        //make these pixels black in the image
                        CV_IMAGE_ELEM(canny_img, uchar, points[c]->point.y, points[c]->point.x) = (uchar)0;
                        if (points[c] != NULL) {
                            free(points[c]);
                            points[c] = NULL;
                        }
                    }
                    points.clear();
                }
            }
        }
    }
    return binRL;
}


void LaneFinder::copy_line_points_Struct(line_points **p_target, line_points* p_src) {
    *p_target = (struct line_points*) malloc(sizeof(struct line_points));
    (*p_target)->point = p_src->point;
    (*p_target)->orientation = p_src->orientation;
    (*p_target)->endpoint = p_src->endpoint;
    (*p_target)->startpoint = p_src->startpoint;
}

void LaneFinder::doJoiningTwoOrient(std::vector< std::vector< struct line_points* > > &binLR,
        std::vector< struct line_points* > &longestLine, int dist) {
    CvPoint endPoint;
    //TODO
    int l = 5;
    double maxAngle = M_PI / 3;
    double minDist = 1000;
    double minDistB = 1000;
    int inx = -1;
    int inxB = -1;
    
    for(int i = 0; i < binLR.size(); i++) {
        if(longestLine.size() > l) {
            CvPoint startPoint = binLR[i][0] ->point;
            CvPoint contPoint = longestLine[longestLine.size() - l]->point;
            endPoint = longestLine[longestLine.size() - 1]->point;
            if (startPoint.y >= endPoint.y) {
                vector< struct line_points* >::iterator iter = binLR[i].begin();
                while((*iter)->point.y >= endPoint.y) {
                    startPoint = (*iter)->point;
                    if((iter + 1) != binLR[i].end()) iter++;
                    else break;
                }
            }
            
            double diff = distBetweenPoints(endPoint, startPoint);
            if (diff < minDist) {
                inx = i;
                minDist = diff;
                double angle = angleBetweenLines(endPoint, contPoint, endPoint, startPoint);
                if (angle == 0 && diff < dist && diff < minDistB) {
                    inxB = i;
                    maxAngle = M_PI/2 + 0.1;
                    minDistB = diff;
                } else {
                    inx = i;
                    minDist = diff;
                    if (angle > maxAngle) maxAngle = angle;
                    inx = i;
                }
            }
        }
    }
    
    int matchingSeg = -1;
    if(minDist <= 2) matchingSeg = inx;
    else if (maxAngle > 0.9) matchingSeg = inxB;
    else if (minDist < dist) {
        CvPoint e1 = longestLine[longestLine.size() - 1]->point;
        CvPoint e2 = longestLine[longestLine.size() - l]->point;
        CvPoint e3 = binLR[inx][binLR[inx].size() - 1]->point;
        double angleTemp = angleBetweenLines(e1, e2, e1, e3);
        if (angleTemp > 0.4) matchingSeg = inx;
    }
    
    if (matchingSeg == -1) return;
    
    int minDist2 = 1000;
    int inx2 = -1;
    for (int i = 0; i < binLR[matchingSeg].size(); i++) {
        double diff2 = distBetweenPoints(binLR[matchingSeg][i]->point, endPoint);
        if (diff2 < minDist2) {
            minDist2 = static_cast<int>(round(diff2));
            inx2 = i;
        }
        if (diff2 == 0) {
            inx2++;
            break;
        }
    }
    
    if(inx2 > 0 && inx2 < binLR[matchingSeg].size()) {
        for (int i = 0; i < binLR[matchingSeg].size() - inx2; i++) {
            struct line_points *p;
            copy_line_points_Struct(&p, binLR[matchingSeg][inx2 + i]);
            longestLine.push_back(p);
        }
    } else {
        for (int i = 0; i < binLR[matchingSeg].size(); i++) {
            struct line_points *p;
            copy_line_points_Struct(&p, binLR[matchingSeg][i]);
            longestLine.push_back(p);
        }
    }
}

void LaneFinder::getTangentParams(std::vector< struct line_points*> &longestLine, int &highest_y_index, double &m,
        double &b, double &angle, CvPoint &startPoint, CvPoint &endPoint, int length, int imgWidth, IplImage *img) {
    int multiplier = 1;
    getHighestYIndex(longestLine, highest_y_index, startPoint);
    
    CvPoint p_1 = startPoint;
    CvPoint p_2 = cvPoint(-1, -1);
    
    int tol = length;
    if (highest_y_index - tol > 0 && highest_y_index - tol < longestLine.size()) {
        p_2 = longestLine[highest_y_index - tol] -> point;
    }
    
    if (p_2.x == -1) {
        if (highest_y_index + tol > 0 && highest_y_index + tol < longestLine.size()) {
            p_2 = longestLine[highest_y_index + tol]->point;
        }
    }
    
    if(p_1.y >= p_2.y) {
        startPoint = p_1;
        endPoint = p_2;
    } else {
        startPoint = p_2;
        endPoint = p_1;
        multiplier = -1;
    }
    
    if (startPoint.x == endPoint.x) {
        if (startPoint.x - 1 > 0 && startPoint.x < imgWidth) startPoint.x -=1;
        else startPoint.x += 1;
    }
    
    Tangent *myTangent = new Tangent(getWidth(), getHeight());
    myTangent -> calTangentParams(startPoint, endPoint, m, b);
    //width = 333, height = 283
    //angle = atan2((getHeight() - endPoint.y) - (getHeight() - startPoint.y), endPoint.x - startPoint.x);
    angle = atan2( 320- endPoint.y, endPoint.x - 166.5);
    angle = (angle/M_PI) * 180;
    angle -= 90;
    angle *= -1;
    if (endPoint.x == -1 && endPoint.y == -1) {
        angle = -1000;
    }
    m *= multiplier;
}

/*Extend the detected line to the bottom of the image using the tangent to the line
 */
void LaneFinder::doExtrapolation(std::vector< struct line_points*> &longestLine, IplImage* img,
        double m, double b, CvPoint &point_y, int lengthTangent) {
    //we wil compute a different start point (on the bottom row of the image)
    longestLine[0]->startpoint = false;
    vector<struct line_points*> tmp_longestLine;
    for (int i = 0; i < longestLine.size(); i++) {
        tmp_longestLine.push_back(longestLine.back());
        longestLine.pop_back();
    }
    
    CvPoint p_1 = point_y;
    int y_start = point_y.y;
    int y_diff = img->height - 1;
    for(int y = y_diff; y > y_start; y--) {
        int x = static_cast<int>(round((y - b) / m));
        if(x > 0 && x < img->width && y > 0) {
            line_points* points = (line_points*)malloc(sizeof(line_points));
            points->point = cvPoint(x, y);
            longestLine.push_back(points);
        }
    }
    for(int i = 0; i < tmp_longestLine.size(); i++) {
        longestLine.push_back(tmp_longestLine.back());
        tmp_longestLine.pop_back();
    }
    tmp_longestLine.clear();
    
		if (longestLine.size() < lengthTangent) {
	    int count = lengthTangent - longestLine.size();
	    y_start = longestLine[longestLine.size() - 1] ->point.y - 1;
	    y_diff = y_start - count;
	    for(int y = y_start; y >= y_diff; y--) {
				int x = static_cast<int>(round((y - b) / m));
				if(x > 0 && x < img->width && y > 0) {
						line_points* points = (line_points*)malloc(sizeof(line_points));
						points->point = cvPoint(x, y);
						longestLine.push_back(points);
				}
	  	}
    }
    point_y = longestLine[longestLine.size() - 1] ->point;
    longestLine[0]->startpoint = true;
}

/**
 * calculates pixel orientations of image img. Useful because edges can be detected
 * by noticing sharp changes in pixel intensities.
 */
IplImage* LaneFinder::getOrientationImage(IplImage *img)
{
    static IplImage* orientationImg = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_32F, 1); 
    static IplImage* sobel_dx = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_16S, 1); 
    static IplImage* sobel_dy = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_16S, 1); 

    //compute the derivatives of the pixel values in the X and Y directions
    cvSobel(img, sobel_dx, 1, 0);
    cvSobel(img, sobel_dy, 0, 1);

    int dx_pic=-1;
    int dy_pic=-1;
    float orientation=-1;
    
    //for each pixel that might be an edge compute the (discrete) orientation of 
    //the tangent at that point w.r.t. pixel intensity
    for(int i = 0; i < orientationImg->width; i++) {
        for(int j = 0; j < orientationImg->height; j++) {
            dy_pic = (short int)(CV_IMAGE_ELEM(sobel_dy, short int, j,i));
            //do this only, if there is an edge (if there is an intensity change)
            if(dx_pic != 0 || dy_pic != 0) {
                if(dx_pic == 0 && dy_pic != 0) {
                    orientation = 90;
                }
                if(dx_pic != 0 && dy_pic != 0) {
                    orientation = atan2((float)(dy_pic), (float)(dx_pic));
                    orientation = ((180 / M_PI) * orientation);
                }
                //now discretize (pixels oriented in a general direction, used for finding lines)
                float temp = orientation;
                if((temp >=0.0 && temp <22.5) || (temp <0.0 && temp >-22.5)) orientation=90;
                if((temp >= 157.5 && temp<= 180.0) || (temp <= -157.5 && temp>= -180.0)) orientation=90;
                if((temp >= 22.5 && temp < 67.5) || (temp <= -112.5 && temp > -157.5)) orientation = 45;
                if((temp >= 67.5  && temp< 112.5)  || (temp <= -67.5  && temp> -112.5)) orientation = 0;
                if((temp >= 112.5 && temp< 157.5) || (temp <= -22.5 && temp> -67.5) ) orientation = 135;
                CV_IMAGE_ELEM(orientationImg, float,j,i)= orientation;
            } else CV_IMAGE_ELEM(orientationImg, float,j,i)= 0;
        }
    }
    
    return orientationImg;
}

/*
* Given an image, a point and orientation data for that image, looks recursively for
* neighbours of that point along lines going left to right
*/
void LaneFinder::getNeighboursOrderedLR(IplImage* img, int x, int y,  std::vector<struct line_points*> &points,
        IplImage* orientationImg, int orientation) {
    //check if next pixel is inside the image
    if ((x - 1 > 0) && (x + 1 < img->width) && (y - 1 > 0) && (y + 1 < img->height)) {
        int pix_val;
        //we started from the bottom row going up and we are looking for (poly)lines going L -> R.
        //Therefore we need to check that the 3 neighbouring pixels (N, W, NW) are on a line,
        //if they are on the same segment (same orientation), or on the same line but the beginning
        //of a new line segment (connected but different orientation)
        
        //check N: x, y-1
        pix_val = CV_IMAGE_ELEM(img, uchar, y-1,x);
        //if there is a line at the next point
        if (pix_val == 255) {
            //check it has the same orientation (on the same line segment)
            if((int)CV_IMAGE_ELEM(orientationImg, float,y-1,x) == orientation) {
                //add the new poingetNeighboursOrderedt to the line
                struct line_points *p;
                p = (struct line_points *) malloc(sizeof(struct line_points));
                p->point = cvPoint(x, y-1);
                float orientation_curr = (float) CV_IMAGE_ELEM(orientationImg, float,y-1,x);
                p->orientation = orientation_curr;
                p->endpoint = false;
                p->startpoint = false;
                points.push_back(p);
                //mark as read
                CV_IMAGE_ELEM(img, uchar, y-1,x)=(uchar)70;
                //continue looking along the line
                getNeighboursOrderedLR(img,x, y-1, points, orientationImg, (int)orientation_curr);
                return;
            }
        }
        
        //check W: x-1, y
        pix_val = CV_IMAGE_ELEM(img, uchar, y, x - 1);
        if (pix_val == 255) {
            if((int)CV_IMAGE_ELEM(orientationImg, float, y, x - 1) == orientation) {
                struct line_points *p;
                p = (struct line_points *) malloc(sizeof(struct line_points));
                p->point = cvPoint(x - 1, y);
                float orientation_curr = (float) CV_IMAGE_ELEM(orientationImg, float, y, x - 1);
                p->orientation = orientation_curr;
                p->endpoint = false;
                p->startpoint = false;
                points.push_back(p);
                //mark as read
                CV_IMAGE_ELEM(img, uchar, y, x - 1)=(uchar)70;
                getNeighboursOrderedLR(img, x - 1, y, points, orientationImg, (int)orientation_curr);
                return;
            }
        }
        
        //check NW: x-1, y-1
        pix_val = CV_IMAGE_ELEM(img, uchar, y - 1, x - 1);
        if (pix_val == 255) {
            if((int)CV_IMAGE_ELEM(orientationImg, float, y - 1, x - 1) == orientation) {
                struct line_points *p;
                p = (struct line_points *) malloc(sizeof(struct line_points));
                p->point = cvPoint(x - 1, y - 1);
                float orientation_curr = (float) CV_IMAGE_ELEM(orientationImg, float, y - 1, x - 1);
                p->orientation = orientation_curr;
                p->endpoint = false;
                p->startpoint = false;
                points.push_back(p);
                //mark as read
                CV_IMAGE_ELEM(img, uchar, y - 1, x - 1)=(uchar)70;
                getNeighboursOrderedLR(img, x - 1, y - 1, points, orientationImg, (int)orientation_curr);
                return;
            }
        }
        
        //we reached the end of a line segment, we need to check if it's
        //connected to a segment with different orientation
        static CvPoint oldPoint = cvPoint(-1, -1);
        static int pointCtr = 0; //keeps track of the different orientations we tried
        if (oldPoint.x == x && oldPoint.y == y) {
            //we've tried this orientation before and failed, try the next one
            pointCtr++;
        } else {
            //first time we've tried new orientations for this point
            pointCtr = 0;
        }
        oldPoint = cvPoint(x,y); //keep track at which point we made the tries
        int newOrientation;
        //cycle through the 4 different orientations until we find a good one or exhaust them
        if(pointCtr < 4) {
            switch(orientation) {
                case 0: 
                    newOrientation = 45;
                    getNeighboursOrderedLR(img,x, y, points, orientationImg, newOrientation);
                    break;

                case 45:
                    newOrientation = 90;
                    getNeighboursOrderedLR(img,x, y, points, orientationImg, newOrientation);
                    break;

                case 90:
                    newOrientation=135;
                    getNeighboursOrderedLR(img,x, y, points, orientationImg, newOrientation);
                    break;

                case 135:
                    newOrientation=0;
                    getNeighboursOrderedLR(img,x, y, points, orientationImg, newOrientation);
                    break;
                    break;
            }
            return;
        } else {
            //there are no other segments, this is the end of the polyline
            points[points.size()-1]->endpoint=true;
            return;
        }
    } else {
        //reached the end of the image
        points[points.size()-1]->endpoint=true;
        return;
    }
}

/*
* Given an image, a point and orientation data for that image, looks recursively for
* neighbours of that point along lines going right to left
*/
void LaneFinder::getNeighboursOrderedRL(IplImage* img, int x, int y,  std::vector<struct line_points*> &points,
        IplImage* orientationImg, int orientation) {
    //check if next pixel is inside the image
    if ((x - 1 > 0) && (x + 1 < img->width) && (y - 1 > 0) && (y + 1 < img->height)) {
        int pix_val;
        //we started from the bottom row going up and we are looking for (poly)lines going R -> L.
        //Therefore we need to check that the 3 neighbouring pixels (N, E, NE) are on a line,
        //if they are on the same segment (same orientation), or on the same line but the beginning
        //of a new line segment (connected but different orientation)
        
        //check E: x + 1, y
        pix_val = CV_IMAGE_ELEM(img, uchar, y, x + 1);
        //if there is a line at the next point
        if (pix_val == 255) {
            //check it has the same orientation (on the same line segment)
            if((int)CV_IMAGE_ELEM(orientationImg, float,y ,x + 1) == orientation) {
                //add the new poingetNeighboursOrderedt to the line
                struct line_points *p;
                p = (struct line_points *) malloc(sizeof(struct line_points));
                p->point = cvPoint(x + 1, y);
                float orientation_curr = (float) CV_IMAGE_ELEM(orientationImg, float,y,x + 1);
                p->orientation = orientation_curr;
                p->endpoint = false;
                p->startpoint = false;
                points.push_back(p);
                //mark as read
                CV_IMAGE_ELEM(img, uchar, y ,x + 1)=(uchar)70;
                //continue looking along the line
                getNeighboursOrderedRL(img,x + 1, y, points, orientationImg, (int)orientation_curr);
                return;
            }
        }
        
        //check N: x, y - 1
        pix_val = CV_IMAGE_ELEM(img, uchar, y - 1, x);
        if (pix_val == 255) {
            if((int)CV_IMAGE_ELEM(orientationImg, float, y - 1, x) == orientation) {
                struct line_points *p;
                p = (struct line_points *) malloc(sizeof(struct line_points));
                p->point = cvPoint(x, y - 1);
                float orientation_curr = (float) CV_IMAGE_ELEM(orientationImg, float, y - 1, x);
                p->orientation = orientation_curr;
                p->endpoint = false;
                p->startpoint = false;
                points.push_back(p);
                //mark as read
                CV_IMAGE_ELEM(img, uchar, y - 1, x)=(uchar)70;
                getNeighboursOrderedRL(img, x, y - 1, points, orientationImg, (int)orientation_curr);
                return;
            }
        }
        
        //check NE: x+1, y-1
        pix_val = CV_IMAGE_ELEM(img, uchar, y - 1, x + 1);
        if (pix_val == 255) {
            if((int)CV_IMAGE_ELEM(orientationImg, float, y - 1, x + 1) == orientation) {
                struct line_points *p;
                p = (struct line_points *) malloc(sizeof(struct line_points));
                p->point = cvPoint(x + 1, y - 1);
                float orientation_curr = (float) CV_IMAGE_ELEM(orientationImg, float, y - 1, x + 1);
                p->orientation = orientation_curr;
                p->endpoint = false;
                p->startpoint = false;
                points.push_back(p);
                //mark as read
                CV_IMAGE_ELEM(img, uchar, y - 1, x + 1)=(uchar)70;
                getNeighboursOrderedRL(img, x + 1, y - 1, points, orientationImg, (int)orientation_curr);
                return;
            }
        }
        
        //we reached the end of a line segment, we need to check if it's
        //connected to a segment with different orientation
        static CvPoint oldPoint = cvPoint(-1, -1);
        static int pointCtr = 0; //keeps track of the different orientations we tried
        if (oldPoint.x == x && oldPoint.y == y) {
            //we've tried this orientation before and failed, try the next one
            pointCtr++;
        } else {
            //first time we've tried new orientations for this point
            pointCtr = 0;
        }
        oldPoint = cvPoint(x,y); //keep track at which point we made the tries
        int newOrientation;
        //cycle through the 4 different orientations until we find a good one or exhaust them
        if(pointCtr < 4) {
            switch(orientation) {
                case 0: 
                    newOrientation = 45;
                    getNeighboursOrderedRL(img,x, y, points, orientationImg, newOrientation);
                    break;

                case 45:
                    newOrientation = 90;
                    getNeighboursOrderedRL(img,x, y, points, orientationImg, newOrientation);
                    break;

                case 90:
                    newOrientation=135;
                    getNeighboursOrderedRL(img,x, y, points, orientationImg, newOrientation);
                    break;

                case 135:
                    newOrientation=0;
                    getNeighboursOrderedRL(img,x, y, points, orientationImg, newOrientation);
                    break;
                    break;
            }
            return;
        } else {
            //there are no other segments, this is the end of the polyline
            points[points.size()-1]->endpoint=true;
            return;
        }
    } else {
        //reached the end of the image
        points[points.size()-1]->endpoint=true;
        return;
    }
}

void LaneFinder::printLane(IplImage* imgColor, const std::vector< struct line_points* > &longestLine) {
    int width = imgColor->width;
    int height = imgColor->height;
    
    CvScalar s;
    s.val[0] = 0; s.val[1] = 255; s.val[2] = 255;
    int length = longestLine.size();
    //cvCircle(imgColor, longestLine[0]->point, 1, s, 5);
    for(int i = 0; i < length; i++) {
        int x = longestLine[i]->point.x;
        int y = longestLine[i]->point.y;
        
        cvCircle(imgColor, cvPoint(x,y), 1, s, 2);
    }
}

void LaneFinder::deleteLine(std::vector< struct line_points*> const &longestLine )
{
    for(int i = 0; i < longestLine.size(); i++) {
        if (longestLine[i] != NULL) free(longestLine[i]);
    }
}

void LaneFinder::extractLane(IplImage *imgInput, IplImage *imgOutput, std::vector<CvPoint> &lineRight,
        int lowerLimit, int higherLimit, int minSize, int lengthTangent, double& alpha) {    
    double* _alpha = &alpha;
    int x_pos;    
    
    vector< vector< struct line_points* > > binLR = binLinesLR(imgInput);        
    vector< vector< struct line_points* > > binRL = binLinesRL(imgInput);
    vector< struct line_points* > longestLine;
    int yLimit = imgInput->height-50;
    
    findLines(imgInput, _alpha, x_pos, longestLine, lowerLimit, higherLimit, yLimit, binLR, binRL, minSize, lengthTangent);
    //debug
    //cvCvtColor(imgInput, imgOutput, CV_GRAY2BGR);
    //printLane(imgOutput, longestLine);
    
    convertLineStructToVector(lineRight, longestLine);
    
    //free memory
    for(int i=0; i<binLR.size();i++) {
        for(int j=0; j<binLR[i].size(); j++) {
            if(binLR[i][j] != NULL) {
                free((binLR[i][j]));
                binLR[i][j]=NULL;
            }
        }
        binLR[i].clear();
    }
    binLR.clear();

    for(int i=0; i<binRL.size();i++) {
        for(int j=0; j<binRL[i].size(); j++) {
            if(binRL[i][j] != NULL) {
                free((binRL[i][j]));
                binRL[i][j]=NULL;
            }
        }
        binRL[i].clear();
    }
    binRL.clear();
    deleteLine(longestLine);
    longestLine.clear();//delete polyCurve
}


/**
 * A scalar Kalman filter, for the interval at the bottom of the image where the lane is thought to be
 * @param state_mean_prev: mean of the previous estimate
 * @param state_var_prev: variance of the previous estimate
 * @param measurement: the measurement
 * @param state_mean_estimate: mean of the calculated estimate
 * @param state_var_estimate: variance of the calculated estimate
 */
void LaneFinder::doKalmanFiltering(double state_mean_prev, double state_var_prev, double measurement,
        double &state_mean_estimate, double &state_var_estimate)
{
    //if the process noise is big, you would rather trust the measurement
    //this is way Q << R
    double Q = 0.01; //state noise variance
    double R = 1000; //measurement noise variance
    
    //Prediction
    double state_mean_pred = state_mean_prev;   //a priori mean estimator
    double state_var_pred = state_var_prev + Q; //a priori variance estimator
    double residual = measurement - state_mean_pred;
    
    //Kalman gain
    double K = state_var_pred / (state_var_pred + R);
    
    //Correction or update
    
    if(fabs(residual) <= 50) {
        //in case the residual is small enough, we had a good prediction so we update
        state_mean_estimate = state_mean_pred + K * residual;
        state_var_estimate = state_var_pred * (1-K);
        
        //the filter is estimating a constant therefore the variance converges to 0.
        //We want however a wider interval where to conduct our line search, therefore
        //limit the interval minimum width
        if(state_var_estimate < 500) state_var_estimate = 500;
    }
    //If nothing is detected the measurement = 0 and the residual is large.
    //In this case we don't want the estimate to shift towards 0, therefore
    //we want to trust the prediction and discard the measurement.
    else if(fabs(residual) > 50) {
        //We might not detect anything because the interval is to small.
        //We can change the measurement variance to counteract this.
        state_var_estimate += 200;
    }
    //if the measurement is way off we discard it completely
    if(fabs(residual) > 200) state_mean_estimate = state_mean_pred;
}

Tangent::Tangent(int width, int height) {
    WIDTH = width;
    HEIGHT = height;
}

Tangent::~Tangent() {
    
}

int Tangent::getWidth() {
    return WIDTH;
}

int Tangent::getHeight() {
    return HEIGHT;
}

void Tangent::calTangentParams(CvPoint& p_1, CvPoint& p_2, double& m, double& b) {
    if (p_1.x == p_2.x) p_2.x -= 1;
    if (p_1.y == p_1.y) p_1.y -= 1;
    
    m = static_cast<float>((getHeight() - p_2.y) - (getHeight() - p_1.y)) / (p_1.x - p_2.x);
    b = static_cast<float>(p_1.y - m * p_1.x);
}

double Tangent::calTangentValue(double m, int x, double b) {
    double y = m * x + b;
    return y;
}
