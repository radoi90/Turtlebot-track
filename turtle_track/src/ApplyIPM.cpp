#include "ApplyIPM.h"

using namespace cv;

//map one channel of BGR image, used for parallel processing
void mapB(const Mat& imgIn, Mat& imgOut, const Mat& pixels,
				const Mat& weights, int nCols, int nRows, int M, int chan) {
	int pi, pj;
	
	for(int i = 0; i < nRows; i++)
		for(int j = 1; j <= nCols; j++) {
			Vec4d wVec = weights.at<Vec4d>(i,j);
			if (!(wVec[0] == 0 && wVec[1] == 0 && wVec[2] == 0 && wVec[3] == 0)) {
				Vec4i iVec = pixels.at<Vec4i>(i,j);
				int pixval = 0;
				for(int k = 0; k < 4; k++) {
					pj = (int)iVec[k] / M;
					pi = iVec[k] % M;
					pixval += imgIn.at<Vec3b>(pi,pj)[chan] * wVec[k];
				}
				imgOut.at<Vec3b>(nCols - j,i)[chan] = pixval;
			}
		}
}

//constructor
ApplyIPM:: ApplyIPM(int m, int n) {
	//input image dimensions
	M = m;
	N = n;
	
	//read mappings
	FileStorage fs1("pixelMap.yml", FileStorage::READ);
	FileStorage fs2("weightMap.yml", FileStorage::READ);	
	fs1["mtx"] >> pixels;
	fs2["mtx"] >> weights;
	
	//output image dimensions
	nRows = pixels.rows;
	nCols = pixels.cols;
}

ApplyIPM:: ~ApplyIPM() {
}

//map a color image
Mat ApplyIPM::mapBGR(const Mat& imgIn) {
	Mat imgOut = Mat::zeros(nCols, nRows, CV_8UC3);
	int pi, pj;
	
	for(int i = 0; i < nRows; i++)
		for(int j = 1; j <= nCols; j++) {
			Vec4i iVec = pixels.at<Vec4i>(i,j);
			Vec4d wVec = weights.at<Vec4d>(i,j);
			Vec3b pixval; pixval[0] = 0; pixval[1] = 0; pixval[2] = 0;
			for(int k = 0; k < 4; k++) {
				pj = (int)iVec[k] / M;
				pi = iVec[k] % M;
				pixval[0] += imgIn.at<Vec3b>(pi,pj)[0] * wVec[k];
				pixval[1] += imgIn.at<Vec3b>(pi,pj)[1] * wVec[k];
				pixval[2] += imgIn.at<Vec3b>(pi,pj)[2] * wVec[k];
			}
			imgOut.at<Vec3b>(nCols - j,i) = pixval;
		}
	
	return imgOut;
}

//map a color image using parallel threads for each of the BGR channels
void ApplyIPM::mapBGRfast(const Mat& imgIn, Mat& imgOut) {
	//imgOut = Mat::zeros(nCols, nRows, CV_8UC3);

	boost::thread t0(boost::bind(&mapB, imgIn, imgOut, pixels, weights, nCols, nRows, M, 0));
	boost::thread t1(boost::bind(&mapB, imgIn, imgOut, pixels, weights, nCols, nRows, M, 1));
	boost::thread t2(boost::bind(&mapB, imgIn, imgOut, pixels, weights, nCols, nRows, M, 2));
	t0.join(); t1.join(); t2.join();
}

//map a greyscale image
Mat ApplyIPM::mapGRAY(const Mat& imgIn) {
	Mat imgOut = Mat::zeros(nCols, nRows, CV_64F);
	int pi, pj;
	
	for(int i = 0; i < nRows; i++)
		for(int j = 1; j <= nCols; j++) {
			Vec4i iVec = pixels.at<Vec4i>(i,j);
			Vec4d wVec = weights.at<Vec4d>(i,j);
			double pixval = 0;
			for(int k = 0; k < 4; k++) {
				pj = (int)iVec[k] / M;
				pi = iVec[k] % M;
				pixval += ((imgIn.at<unsigned char>(pi,pj) *1.0) / 255) * wVec[k];
			}
			imgOut.at<double>(nCols - j,i) = pixval;
		}
		
	return imgOut;
}
