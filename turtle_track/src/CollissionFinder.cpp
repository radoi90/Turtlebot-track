#include "CollissionFinder.h"

void fillRoadColor(Mat& rgbWorFrame, Vec3b roadColor) {
	for (int i = 0; i < rgbWorFrame.cols; i++)
			for (int j = 0; j < rgbWorFrame.rows; j++) {
				rgbWorFrame.at<Vec3b>(j,i)[0] = roadColor[0];
				rgbWorFrame.at<Vec3b>(j,i)[1] = roadColor[1];
				rgbWorFrame.at<Vec3b>(j,i)[2] = roadColor[2];
			}
}

Vec3b extractRoadColor(Mat img) {
    Vec3b roadColor;
    //average over a square immediately in front of the robot
    //upper left corner at (x,y), side length boxWidth
    int boxWidth = 30;
    int boxLength = 50;
    int x = ((int) img.cols / 3);
    int y = img.rows - boxLength - 10;
    int r, g, b;
    r = g = b = 0;
    
    //find the average in the area of interest
    for (int i = 0; i < boxLength; i++) 
        for(int j = 0; j < boxWidth; j++) {
            Vec3b pixel = img.at<Vec3b>(y + i, x + j);
            
            //debug: remove area of interest from image to see where it is
            img.at<Vec3b>(y + i, x + j)[0] = img.at<Vec3b>(y + i, x + j)[1] = img.at<Vec3b>(y + i, x + j)[2] = 0;
            
            if (pixel[0] <= 255) r += pixel[0];
            if (pixel[1] <= 255) g += pixel[1];
            if (pixel[2] <= 255) b += pixel[2];
        }
    
    r /= boxWidth * boxLength;
    g /= boxWidth * boxLength;
    b /= boxWidth * boxLength;
    
    roadColor[0] = (uchar) r;
    roadColor[1] = (uchar) g;
    roadColor[2] = (uchar) b;
    
    return roadColor;
}

uchar isCollisionBin(Vec3b pixel, Vec3b color, int threshold) {
    Vec3b collisionPixel;
    
    for(int i = 0; i < 3; i++)
        if (abs(pixel[i] - color[i]) > threshold) return 255;
    
    return 0;
}

Vec3b isCollision(Vec3b pixel, Vec3b color, int threshold) {
    Vec3b collisionPixel;//bgr
    
    for(int i = 0; i < 3; i++)
        collisionPixel[i] = (abs(pixel[i] - color[i]) <= threshold) ? 0 : 255;

    return collisionPixel;
}

void findCollisions(Mat img, Mat &binaryImg, Vec3b color, int threshold) {    
    for(int i = 0; i < img.rows; i++)
        for(int j = 0; j < img.cols; j++) {
            binaryImg.at<Vec3b>(i,j) = isCollision(img.at<Vec3b>(i,j), color, threshold);
        }
}


void findCollisionsBin(Mat img, Mat &binaryImg, Vec3b color, int threshold) {
    for(int i = 0; i < img.rows; i++)
        for(int j = 0; j < img.cols; j++) {
            binaryImg.at<uchar>(i,j) = isCollisionBin(img.at<Vec3b>(i,j), color, threshold);
        }
}

int avoidCollision(Mat colWorImg, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo) {
	int widthFrom, widthTo, index=0;
	//get to first row of interest
	while (index < lane.size() && (colWorImg.rows - lane[index].y) < distFrom) index++;
	//check each row in the region of interest, in order of increasing distance
	while (index < lane.size() && (colWorImg.rows - lane[index].y) < distTo) {
		//check pixels along the lane, where the robot will pass through
		widthFrom = std::max(0,lane[index].x - ((ROBOT_WIDTH + LINE_WIDTH) / 2));
		widthTo = std::min(colWorImg.cols-1, widthFrom + ROBOT_WIDTH + LINE_WIDTH);
		//debug
		colWorImg.at<Vec3b>(lane[index].y, widthFrom)[2] = 255;
		colWorImg.at<Vec3b>(lane[index].y, widthTo)[2] = 255;
		int nonRoad = 0; //number of non-road pixels
		//check the lane
		for(int i = widthFrom; i < widthTo; i++) {
			Vec3b p = colWorImg.at<Vec3b>(lane[index].y, i);
			if ((p[0] + p[1] + p[2]) > 0) nonRoad++;
			if (nonRoad >= threshold) {
				//debug
				line(colWorImg, Point(widthFrom, lane[index].y), Point(widthTo, lane[index].y), Scalar(0,255,0), 1);
				return index;
			}
		}
		index++; 
	}

	return colWorImg.rows;
}

int avoidCollisionBin(Mat colWorImg, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo) {
	int widthFrom, widthTo, index=0;
	//get to first row of interest
	while (index < lane.size() && (colWorImg.rows - lane[index].y) < distFrom) index++;
	//check each row in the region of interest, in order of increasing distance
	while (index < lane.size() && (colWorImg.rows - lane[index].y) < distTo) {
		//check pixels along the lane, where the robot will pass through
		widthFrom = std::max(0,lane[index].x - ((ROBOT_WIDTH + LINE_WIDTH) / 2));
		widthTo = std::min(colWorImg.cols-1, widthFrom + ROBOT_WIDTH + LINE_WIDTH);
		//debug
		colWorImg.at<uchar>(lane[index].y, widthFrom) = 75;
		colWorImg.at<uchar	 >(lane[index].y, widthTo) = 75;
		int nonRoad = 0; //number of non-road pixels
		//check the lane
		for(int i = widthFrom; i < widthTo; i++) {
			uchar p = colWorImg.at<uchar>(lane[index].y, i);
			if (p > 0) nonRoad++;
			if (nonRoad >= threshold) {
				//debug
				line(colWorImg, Point(widthFrom, lane[index].y), Point(widthTo, lane[index].y), Scalar(150), 1);
				return index;
			}
		}
		index++; 
	}

	return colWorImg.rows;
}

int avoidCollisionBinUpdate(Mat colWorImg, Mat rgbWorFrame, Vec3b& roadColor, std::vector<CvPoint> lane, int threshold, int distFrom, int distTo) {
	int widthFrom, widthTo, index=0;
	int b, g, r, count;
	b=g=r=count=0;
	//get to first row of interest
	while (index < lane.size() && (colWorImg.rows - lane[index].y) < distFrom) index++;
	//check each row in the region of interest, in order of increasing distance
	while (index < lane.size() && (colWorImg.rows - lane[index].y) < distTo) {
		//check pixels along the lane, where the robot will pass through
		widthFrom = std::max(0,lane[index].x - ((ROBOT_WIDTH + LINE_WIDTH) / 2));
		widthTo = std::min(colWorImg.cols-1, widthFrom + ROBOT_WIDTH + LINE_WIDTH);
		//debug
		colWorImg.at<uchar>(lane[index].y, widthFrom) = 75;
		colWorImg.at<uchar>(lane[index].y, widthTo) = 75;
		int nonRoad = 0; //number of non-road pixels
		//check the lane
		for(int i = widthFrom; i < widthTo; i++) {
			uchar p = colWorImg.at<uchar>(lane[index].y, i);
			if (p > 0) nonRoad++;
			else {
				b += rgbWorFrame.at<Vec3b>(lane[index].y, i)[0];
				g += rgbWorFrame.at<Vec3b>(lane[index].y, i)[1];
				r += rgbWorFrame.at<Vec3b>(lane[index].y, i)[2];
				count++;
			}
			if (nonRoad >= threshold) {
				//debug
				line(colWorImg, Point(widthFrom, lane[index].y), Point(widthTo, lane[index].y), Scalar(150), 1);
				return (colWorImg.rows - lane[index].y);
			}
		}
		//if the road was clear enough update the road color
		if (index == 20) {
			roadColor[0] = (uchar) (b/count);
		    roadColor[1] = (uchar) (g/count);
		    roadColor[2] = (uchar) (r/count);
			if(isCollisionBin(rgbWorFrame.at<Vec3b>(rgbWorFrame.rows - 1, 0), roadColor, 35) > 0)
				fillRoadColor(rgbWorFrame, roadColor);
		}
		index++;
	}

	return colWorImg.rows;
}
