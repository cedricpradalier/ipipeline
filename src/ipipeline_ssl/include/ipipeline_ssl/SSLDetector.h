#ifndef __SSL_DETECTOR_HPP_
#define __SSL_DETECTOR_HPP_

#include "opencv2/core/core.hpp"
#include <vector>

#include "ipipeline_ssl/SSL.h"

typedef std::vector<SSL> ListSSL;

class ROI
{
public:
	ROI() : x(0), y(0),
			width(width), height(height) {};
	ROI(unsigned int x, unsigned int y,
		unsigned int width, unsigned int height)
		: x(x), y(y), width(width), height(height) {};
	unsigned int x;
	unsigned int y;
	unsigned int width;
	unsigned int height;
};

class SSLDetector
{
public:
	static const unsigned int SSL_MAX_SCALE = 300;
	
private:
	unsigned int w_, linestep_;
	int *idxp_;
	int *idxsqrtp_;
    cv::Mat
	
public:
	SSLDetector();
	SSLDetector(const cv::Mat &vid, unsigned int linestep=1,
				unsigned int w=30, double p=2./3.);
	~SSLDetector();
	
	void init( const cv::Mat &vid, unsigned int linestep=1,
			   unsigned int w=30, double p=2./3. );
	
	bool track(const cv::Mat &vid, const ROI& roi, SSL& ssl,
			   short threshold=100);
	void detectAll(const cv::Mat &vid, const ROI& roi,
				   ListSSL &listSSL, short threshold=1000);

	void computeAll(const cv::Mat &vid, ListSSL &listSSL, short threshold=1000);
	
	unsigned int computeScale(const cv::Mat &vid,
							  unsigned int x, unsigned int y);
	
	inline const short* getBuf() { return sslbuf_;}
	inline unsigned int getW() { return w_; }
	inline unsigned int getLineStep() { return linestep_;}
};

#endif // __SSL_DETECTOR_HPP_
