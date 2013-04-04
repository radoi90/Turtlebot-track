#ifndef APPLYIPM_H
#define	APPLYIPM_H
#include <stdlib.h>
#include <cv.h>
#include <boost/thread.hpp>

class ApplyIPM {
private:
	//dimensions of input image
	int M;
	int N;
	//pixel and weight maps
	cv::Mat pixels;
	cv::Mat weights;
	
public:
	//contstructor
	ApplyIPM(int m, int n);
	//destructor
	~ApplyIPM();
	
	//dimensions of output image
	int nRows;
	int nCols;
	
	cv::Mat mapBGR(const cv::Mat& imgIn);
	cv::Mat mapGRAY(const cv::Mat& imgIn);
	void mapBGRfast(const cv::Mat& imgIn, cv::Mat& imgOut);
};

#endif
