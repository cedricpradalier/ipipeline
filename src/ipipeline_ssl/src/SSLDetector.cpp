#include "SSLDetector.hpp"

#include <iostream>
#include <cmath>
#include <algorithm>

#include <fstream>

using namespace std;

const unsigned int SSLDetector::SSL_MAX_SCALE;

SSLDetector::SSLDetector()
{
}

SSLDetector::SSLDetector( IplImage *vid, unsigned int linestep, unsigned int w, double p)
{
	init(vid, linestep, w, p);
}

void SSLDetector::init(IplImage *vid, unsigned int linestep, unsigned int w, double p)
{
	w_ = w;
	linestep_=linestep;
	
	idxsqrtp_ = new int[SSL_MAX_SCALE];
	idxp_ = new int[SSL_MAX_SCALE];
	sslbuf_ = new short[vid->width*(vid->height/linestep)];
	
	double sqrtp = sqrt(p);
	
	for(unsigned int i=1; i<SSL_MAX_SCALE; i++){
		idxp_[i] = (int) round(p*i);
		idxsqrtp_[i] = (int) round(sqrtp*i);
	}	
}

SSLDetector::~SSLDetector()
{
	delete[] idxsqrtp_;
	delete[] idxp_;
	delete[] sslbuf_;
}

bool SSLDetector::track( IplImage *vid, const ROI& roi, SSL& ssl, short threshold)
{
	unsigned int width = vid->width;
	//unsigned int height = vid->height;
	unsigned char *imgptr = (unsigned char*)vid->imageData + width*roi.y+roi.x;
	//short *sslptr = sslbuf_;
	
	short maxval = threshold;
	unsigned int xmax=0;
	unsigned int ymax=0;
	
	for(unsigned int y=0;
		y<roi.height;
		y+=linestep_, imgptr+=linestep_*width)
	{
		const unsigned char *imgptr2 = imgptr;
		for(unsigned int x=0;
			x<roi.width;
			x++, /*sslptr++,*/ imgptr2++)
		{
			unsigned short s0=0;
			unsigned short s1=0;
			
			short res;
			unsigned short i;
			unsigned short bound = min(w_, width-x);
			
			for(i=1; i<bound; i++)
			{
				unsigned char v = imgptr2[i];
				int idx1 = idxp_[i];
				int idx2 = idxsqrtp_[i];
				s0 += abs( imgptr2[idx1] - v );
				s1 += abs( imgptr2[idx2] - v );
			}
			
			res = s1-s0;
			
			s1=s0=0;
			
 			unsigned short bound2 = min(w_, x);
			
			for(i=1; i<bound2; i++)
			{
				unsigned char v = imgptr2[-i];
				s0 += abs( imgptr2[-idxp_[i]] - v );
				s1 += abs( imgptr2[-idxsqrtp_[i]] - v );
			}
			res += s1-s0;
			res = (res<<8)/(bound+bound2);

			if(res>maxval)
			{
				maxval = res;
				xmax = x;
				ymax = y;
			}
			
			//*sslptr = (res<<8)/(bound+bound2);
		}
	}
	
	if(maxval > threshold)
	{
		ssl.X()=xmax+roi.x;
		ssl.Y()=ymax+roi.y;
		ssl.Value()=maxval;
		return true;
	}
	return false;
}

void SSLDetector::detectAll( IplImage *vid, const ROI& roi, ListSSL& listSSL, short threshold)
{
	unsigned int width = vid->width;
	unsigned int height = vid->height;
	unsigned char *imgptr = (unsigned char*)vid->imageData + width*roi.y+roi.x;
	short *sslptr = sslbuf_;
		
	for(unsigned int y=0;
		y<roi.height;
		y+=linestep_, imgptr+=linestep_*width)
	{
		const unsigned char *imgptr2 = imgptr;
		for(unsigned int x=0;
			x<roi.width;
			x++, sslptr++, imgptr2++)
		{
			unsigned short s0=0;
			unsigned short s1=0;
			
			short res;
			unsigned short i;
			unsigned short bound = min(w_, width-x);
			
			for(i=1; i<bound; i++)
			{
				unsigned char v = imgptr2[i];
				int idx1 = idxp_[i];
				int idx2 = idxsqrtp_[i];
				s0 += abs( imgptr2[idx1] - v );
				s1 += abs( imgptr2[idx2] - v );
			}
			
			res = s1-s0;
			
			s1 = s0=0;
			
 			unsigned short bound2 = min(w_, x);
			
			for(i=1; i<bound2; i++)
			{
				unsigned char v = imgptr2[-i];
				s0 += abs( imgptr2[-idxp_[i]] - v );
				s1 += abs( imgptr2[-idxsqrtp_[i]] - v );
			}
			res += s1-s0;
			
			// vertical ssl
 			unsigned short bound3 = min(w_, y + roi.y);
			s1 = s0 = 0;
			for(i=1; i<bound3; i++)
			{
				unsigned char v = imgptr2[-i*width];
				s0 += abs( imgptr2[-idxp_[i]*width] - v );
				s1 += abs( imgptr2[-idxsqrtp_[i]*width] - v );
			}
			res += s1-s0;

			unsigned short bound4 = min(w_, height - y - roi.y);
			s1 = s0 = 0;
			
			for(i=1; i<bound4; i++)
			{
				unsigned char v = imgptr2[i*width];
				s0 += abs( imgptr2[idxp_[i]*width] - v );
				s1 += abs( imgptr2[idxsqrtp_[i]*width] - v );
			}
			res += s1-s0;
			
			*sslptr = (res<<8)/(/*bound+bound2+*/bound3+bound4);
		}
	}
	
	sslptr = sslbuf_;

	//int n=0;
	
	for(unsigned int y=0;
		y<roi.height/linestep_;
		y++)
	{
		for( unsigned int x=0;
			 x<roi.width;
			 x++, sslptr++)
		{
			short val = *sslptr;
			if(val>threshold)
			{
				bool b = true;
				for(int j=max(-5, -(int)y);
					b && j<=min((int)(roi.height/linestep_-1-y),5);
					j++){
					int p = j*roi.width;
					for(int i=max(-5, -(int)x);
						b && i<=min((int)(roi.width-1-x),5);
						i++){
						b = val>=sslptr[i+p];
					}
				}
				if(b){
					listSSL.push_back(SSL(x+roi.x, linestep_*y+roi.y, val));
				}
			}
		}
	}
}

unsigned int SSLDetector::computeScale(IplImage *vid,
									   unsigned int x,
									   unsigned int y)
{
	unsigned int width = vid->width;
	unsigned int height = vid->height;
	unsigned char *imgptr = (unsigned char*)vid->imageData + width*y+x;
	
	float res=0;
	float max=0;
	unsigned int  smax=0;
	//unsigned short i;
	unsigned int bound = min(SSL_MAX_SCALE, width-x);

	unsigned short s0=0;
	unsigned short s1=0;

	//ofstream file("scale.log");
	
	for(unsigned int i=1; i<bound; i++)
	{
		unsigned char v = imgptr[i];
		int idx1 = idxp_[i];
		int idx2 = idxsqrtp_[i];
		s0 += abs( imgptr[idx1] - v );
		s1 += abs( imgptr[idx2] - v );
		
		res += s1-s0;

		float res2 = res/(i*i*sqrtf(i));
		if(res2 >= max ){
			max = res2;
			smax = i;
		}
		//file<<i<<" "<<res<<" "<<res2<<std::endl;
	}
	//file.close();

	// now in the other side
	res=0;
	bound = min(SSL_MAX_SCALE, x);
	s0=0;
	s1=0;
	
	for(unsigned int i=1; i<bound; i++)
	{
		unsigned char v = imgptr[-i];
		int idx1 = idxp_[i];
		int idx2 = idxsqrtp_[i];
		s0 += abs( imgptr[-idx1] - v );
		s1 += abs( imgptr[-idx2] - v );
		
		res += s1-s0;
		float res2 = res/(i*i*sqrtf(i));
		if(res2 >= max ){
			max = res2;
			smax = i;
		}
	}
	
	// now in the vertical side
	res=0;
	bound = min(SSL_MAX_SCALE, y);
	s0=0;
	s1=0;
	
	for(unsigned int i=1; i<bound; i++)
	{
		unsigned char v = imgptr[-i];
		int idx1 = idxp_[i];
		int idx2 = idxsqrtp_[i];
		s0 += abs( imgptr[-idx1*width] - v );
		s1 += abs( imgptr[-idx2*width] - v );
		
		res += s1-s0;
		float res2 = res/(i*i*sqrtf(i));
		if(res2 >= max ){
			max = res2;
			smax = i;
		}
	}

	res=0;
	bound = min(SSL_MAX_SCALE, height-y);
	s0=0;
	s1=0;
	
	for(unsigned int i=1; i<bound; i++)
	{
		unsigned char v = imgptr[-i];
		int idx1 = idxp_[i];
		int idx2 = idxsqrtp_[i];
		s0 += abs( imgptr[-idx1*width] - v );
		s1 += abs( imgptr[-idx2*width] - v );
		
		res += s1-s0;
		float res2 = res/(i*i*sqrtf(i));
		if(res2 >= max ){
			max = res2;
			smax = i;
		}
	}
	
	return smax;
}

void SSLDetector::computeAll(IplImage *vid, ListSSL &listSSL, short threshold)
{
	unsigned int width = vid->width;
	unsigned int height = vid->height;
	unsigned char *imgptr = (unsigned char*)vid->imageData;
	short *sslptr = sslbuf_;
		
	for(unsigned int y=0;
		y<height;
		y+=linestep_, imgptr+=linestep_*width)
	{
		const unsigned char *imgptr2 = imgptr;
		for(unsigned int x=0;
			x<width;
			x++, sslptr++, imgptr2++)
		{
			unsigned short s0=0;
			unsigned short s1=0;
			
			short res;
			unsigned short i;
			unsigned short bound = min(w_, width-x);
			
			for(i=1; i<bound; i++)
			{
				unsigned char v = imgptr2[i];
				int idx1 = idxp_[i];
				int idx2 = idxsqrtp_[i];
				s0 += abs( imgptr2[idx1] - v );
				s1 += abs( imgptr2[idx2] - v );
			}
			
			res = s1-s0;
			
			s1 = s0=0;
			
 			unsigned short bound2 = min(w_, x);
			
			for(i=1; i<bound2; i++)
			{
				unsigned char v = imgptr2[-i];
				s0 += abs( imgptr2[-idxp_[i]] - v );
				s1 += abs( imgptr2[-idxsqrtp_[i]] - v );
			}
			res += s1-s0;
			
			// vertical ssl
 			unsigned short bound3 = min(w_, y);
			s1 = s0 = 0;
			for(i=1; i<bound3; i++)
			{
				unsigned char v = imgptr2[-i*width];
				s0 += abs( imgptr2[-idxp_[i]*width] - v );
				s1 += abs( imgptr2[-idxsqrtp_[i]*width] - v );
			}
			res += s1-s0;

			unsigned short bound4 = min(w_, height - y);
			s1 = s0 = 0;
			
			for(i=1; i<bound4; i++)
			{
				unsigned char v = imgptr2[i*width];
				s0 += abs( imgptr2[idxp_[i]*width] - v );
				s1 += abs( imgptr2[idxsqrtp_[i]*width] - v );
			}
			res += s1-s0;
			
			*sslptr = (res<<8)/(/*bound+bound2+*/bound3+bound4);
		}
	}
	
	sslptr = sslbuf_;

	//int n=0;
	
	for(unsigned int y=0;
		y<height/linestep_;
		y++)
	{
		for( unsigned int x=0;
			 x<width;
			 x++, sslptr++)
		{
			short val = *sslptr;
			if(val>threshold)
			{
				bool b = true;
				for(int j=max(-5, -(int)y);
					b && j<=min((int)(height/linestep_-1-y),5);
					j++){
					int p = j*width;
					for(int i=max(-5, -(int)x);
						b && i<=min((int)(width-1-x),5);
						i++){
						b = val>=sslptr[i+p];
					}
				}
				if(b){
					unsigned int scale = computeScale(vid, x, linestep_*y);
					listSSL.push_back(SSL(x, linestep_*y, val, scale));
				}
			}
		}
	}

	
	
}
