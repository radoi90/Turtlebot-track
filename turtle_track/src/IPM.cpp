//Based on the work of Eric Johnson, Randy Hamburger
//Original code at http://www.eng.utah.edu/~hamburge/CVprojectCode/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>

//input image size
#define M 480
#define N 640
#define pi 3.14159265359
//netbook on top of turtle bot
//#define h 0.57
//#define theta0 0.637045177
//webcam on second shelf on book
#define h 0.253
#define theta0 0.333

using namespace cv;
using namespace std;

Vec2i getRowBounds(int xRow, int mCropped) {
	int r12, r34;
	if (xRow < mCropped) {
		r12 = xRow;
		r34 = xRow - 1;
	} else {
		r12 = xRow - 1;
		r34 = xRow - 2;
	}
	
	Vec2i v = Vec<int,2>(r12, r34);
	return v;
}

Vec4i getColBounds(double y, Mat Yvis, int r12, int r34) {
	Vec4i cVec = Vec<int, 4>(0,0,0,0);
	
	Mat yVec12 = Yvis.row(r12);
	int yCol12 = 0;
	
	int i;
	if (y >= yVec12.at<double>(0, yVec12.cols - 1))
		for(i = 0; i < yVec12.cols; i++)
			yCol12 += (yVec12.at<double>(0, i) >= y);
	
	if (yCol12 > 0) {
		Mat yVec34 = Yvis.row(r34);
		int yCol34 = 0;
		
		if (y >= yVec34.at<double>(0, yVec34.cols - 1))
			for(i = 0; i < yVec34.cols; i++)
				yCol34 += (yVec34.at<double>(0, i) >= y);
		
		if (yCol34 > 0) {
			if (yCol12 < yVec12.cols) {
				cVec[0] = yCol12;
				cVec[1] = yCol12 + 1;
			} else {
				cVec[0] = yCol12 - 1;
				cVec[1] = yCol12;
			}
			
			if (yCol34 < yVec34.cols) {
				cVec[2] = yCol34;
				cVec[3] = yCol34 + 1;
			} else {
				cVec[2] = yCol34 - 1;
				cVec[3] = yCol34;
			}
		}
	}
	
	return cVec;	
}

Vec4i getIndices(int m, int mCropped, int n, Vec4i rVec, Vec4i cVec) {
	Vec4i iVec = Vec<int, 4>();
	int shift = m - mCropped;
	Vec4i rOrig = Vec<int, 4>(rVec[0] + shift, rVec[1] + shift, rVec[2] + shift, rVec[3] + shift);
	
	iVec[0] = cVec[0] * m + rOrig[0];
	iVec[1] = cVec[1] * m + rOrig[1];
	iVec[2] = cVec[2] * m + rOrig[2];
	iVec[3] = cVec[3] * m + rOrig[3];
	
	return iVec;
}

Vec4d getWeights(double x, double y, Vec4i rVec, Vec4i cVec, Mat Xvis, Mat Yvis) {
	Vec4d wVec = Vec<double, 4>();
	double px[4], py[4];
	int p;
	
	for(p = 0; p < 4; p++) {
		px[p] = Xvis.at<double>(rVec[p], cVec[p]);
		py[p] = Yvis.at<double>(rVec[p], cVec[p]);
	}
	
	double d12y = py[0] - py[1];
	double d1y = py[0] - y;
	double d2y = y - py[1];
    
	double d34y = py[2] - py[3];
	double d3y = py[2] - y;
	double d4y = y - py[3];
    
	double d13x = px[2] - px[0];
	double d3x = px[2] - x;
	double d1x = x - px[0];
    
	wVec[0] = d3x*d2y/(d13x*d12y);
	wVec[1] = d3x*d1y/(d13x*d12y);
	wVec[2] = d1x*d4y/(d13x*d34y);
	wVec[3] = d1x*d3y/(d13x*d34y);
	
	return wVec;
}

int main(int argc, char* argv[]) {
	double m = M * 1.0;
	double n = N * 1.0;
	double alpha_tot = 30*pi/180;
	double den = sqrt((m-1)*(m-1)+(n-1)*(n-1));
	double alpha_u = atan( (n-1)/den * tan(alpha_tot) );
	double alpha_v = atan( (m-1)/den * tan(alpha_tot) );
	
	//debug
	//Mat imgIn = imread("roomSetup.jpg", 0);
	
	double rHorizon = ceil( (m-1)/2*(1 - tan(theta0)/tan(alpha_v)) + 1 );
	if (rHorizon < 0) rHorizon = 0;
	
	double mCropped = m-rHorizon+1;
	cout << mCropped << "\n";
	Mat Xvis = Mat::zeros(mCropped, N, CV_64FC2);
	Mat Yvis = Mat::zeros(mCropped, N, CV_64FC2);
	
	//debug
	//Rect myROI(0, rHorizon , n, m-rHorizon);
	//Mat imgCropped = imgIn(myROI);
	//imwrite("croppedIPMtest.jpg", imgCropped);
	
	int r;
	for(r = 0; r < mCropped; r++) {
		double rOrig = (r + rHorizon - 1) * 1.0;
		double rFactor = (1-2*(rOrig-1)/(m-1))*tan(alpha_v);
		double num = 1 + rFactor*tan(theta0);
		double den = tan(theta0) - rFactor;

		double val = h*(num/den);
    	
    	int c;
		for(c = 0; c < n; c++) {
			Xvis.at<double>(r,c) = val;
			num = (1-2*(c-1)/(n-1))*tan(alpha_u);
			den = sin(theta0) - rFactor*cos(theta0);
			Yvis.at<double>(r,c) = h*(num/den);
		}
	}
	
	//Output image range (size)
	double step = 0.003;
	double xRange1 = 0.35;
	double xRange2 = 1.2;
	double yRange1 = -0.5;
	double yRange2 = 0.5;
	int nRows = (yRange2 - yRange1)/step;
	int nCols = (xRange2 - xRange1)/step;
	
	Mat xGrid = Mat(1, nCols, CV_64F);
	Mat yGrid = Mat(nRows, 1, CV_64F);

	int i, j;
	double val;
	for(i = 0, val = xRange1; i < nCols; i++, val += step)
		xGrid.at<double>(0, i) = val;
	for(i = 0, val = yRange2; i < nRows; i++, val -= step)
		yGrid.at<double>(i,0) = val;
		
	Mat pixels(nRows, nCols, CV_32SC4);
	for (i = 0; i < nRows; i++)
		for(j = 0; j < nCols; j++) {
			pixels.at<Vec4i>(i,j)[0] = 1;
			pixels.at<Vec4i>(i,j)[1] = 1;
			pixels.at<Vec4i>(i,j)[2] = 1;
			pixels.at<Vec4i>(i,j)[3] = 1;
		}
	Mat weights = Mat::zeros(nRows, nCols, CV_64FC4);
	
	Mat xVec = Xvis.col(0);
	double xMinVis = xVec.at<double>(xVec.rows -1);
	
	int c;
	for(c = 0; c < nCols; c++) {
		double x = xGrid.at<double>(0,c);
		int xRow = 0;
		if (x >= xMinVis) 
			for(i = 0; i < xVec.rows; i++) xRow += (xVec.at<double>(i) >= x);
		
		if (xRow > 0) {
			Vec2i r1234 = getRowBounds(xRow, mCropped);
			int r12 = r1234[0];
			int r34 = r1234[1];
			
			for(r = 0; r < nRows; r++) {
				double y = yGrid.at<double>(r,0);
				Vec4i cVec = getColBounds(y, Yvis, r12, r34);
				
				if (cVec[0] > 0) {
					Vec4i rVec = Vec<int, 4>(r12, r12, r34, r34);
					Vec4i iVec = getIndices(M, mCropped, N, rVec, cVec);
					Vec4d wVec = getWeights(x, y, rVec, cVec, Xvis, Yvis);

					pixels.at<Vec4i>(r,c) = iVec;
					weights.at<Vec4d>(r,c) = wVec;
				}
			}
		}
		if (c % 20 == 0) cout << c << "\n";
	}
	
	//save weights to file
	FileStorage fs1("pixelMap.yml", FileStorage::WRITE);
	fs1 << "mtx" << pixels;
	
	FileStorage fs2("weightMap.yml", FileStorage::WRITE);
	fs2 << "mtx" << weights;
	
	//debug
	//namedWindow("myWindow", CV_WINDOW_AUTOSIZE);
	//imshow("myWindow", imgCropped);
	//waitKey(20000);
	//destroyWindow( "myWindow" );
   
   return 0;
	
}
