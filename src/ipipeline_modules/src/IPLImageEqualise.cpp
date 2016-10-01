
#include "ipipeline_modules/IPLImageEqualise.h"

IPLImageEqualise::IPLImageEqualise(const char * n) : 
	IPLImageFilter(n)
{
}

bool IPLImageEqualise::checkInput() const 
{
	IPLImageProcessor * input = getInput(deftInput);
	const ImageProcessorOutput * in = input->getOutput();
	if (!in->checkType()) return error(INCONSISTENT_TYPE);
	switch (in->type()) {
		case RGB8u:
		case Int8u:
			break;
		default : 
			return error(INVALID_TYPE);
	}
	return true;
}

#define GrayLevels 256
bool IPLImageEqualise::processInput()
{
	IPLImageProcessor * input = getInput(deftInput);
	const IppiImage * in = input->getOutput()->asImage();
	unsigned int k;

	reallocate(in,input->nimages);

	switch (in->type()) {
		case Int8u :
			{
				const IppiImage8u * I = in->as8u();
				const IppiImage8u * O = out->as8u();
				for (k=0;k<nimages;k++) {
					Ipp32s histo[GrayLevels+1];
					Ipp32f s[GrayLevels+1];
					Ipp32s levels[GrayLevels+1], values[GrayLevels+1];

					// calculate histogram
					ippiHistogramEven_8u_C1R(I[k].pixels, I[k].bstep, I[k].size,
							histo, levels, GrayLevels+1, 0, GrayLevels);

					for (int i = 0; i < GrayLevels; i ++) {
						s[i] = float(histo[i]) / I[k].size.width / I[k].size.height;
						if (i > 0) {
							s[i] = s[i] + s[i-1];
						}
						values[i] = Ipp32s(s[i] * (GrayLevels-1));
					}
					values[GrayLevels] = GrayLevels;

					//LUT
					ippiLUT_8u_C1R(I[k].pixels, I[k].bstep, 
							O[k].pixels, O[k].bstep, O[k].size, values, levels, GrayLevels+1);
						                  
				}
				break;
			}
		case RGB8u :
			{
				const IppiImageRGB8u * I = in->asRGB8u();
				const IppiImageRGB8u * O = out->asRGB8u();
				for (k=0;k<nimages;k++) {
					Ipp32s histo1[GrayLevels+1],histo2[GrayLevels+1],histo3[GrayLevels+1];
					Ipp32s levels1[GrayLevels+1],levels2[GrayLevels+1],levels3[GrayLevels+1];
					Ipp32s values1[GrayLevels+1],values2[GrayLevels+1],values3[GrayLevels+1];
					Ipp32s *histo[3] = {histo1,histo2,histo3};
					Ipp32s *levels[3] = {levels1,levels2,levels3};
					Ipp32s *values[3] = {values1,values2,values3};
					Ipp32f s1[GrayLevels+1],s2[GrayLevels+1],s3[GrayLevels+1];
					Ipp32f *s[3] = {s1,s2,s3};
					int nlevels[3] = {GrayLevels,GrayLevels,GrayLevels};
					int nlevels1[3] = {GrayLevels+1,GrayLevels+1,GrayLevels+1};

					// calculate histogram
					ippiHistogramEven_8u_C3R(I[k].pixels, I[k].bstep, I[k].size,
							histo, levels, nlevels1, 0, nlevels);

					for (int j = 0; j < 3; j ++) {
						for (int i = 0; i < GrayLevels; i ++) {
							s[j][i] = float(histo[j][i]) / I[k].size.width / I[k].size.height;
							if (i > 0) {
								s[j][i] = s[j][i] + s[j][i-1];
							}
							values[j][i] = Ipp32s(s[j][i] * (GrayLevels-1));
						}
						values[j][GrayLevels] = GrayLevels;
					}

					//LUT
					ippiLUT_8u_C3R(I[k].pixels, I[k].bstep, 
							O[k].pixels, O[k].bstep, O[k].size, (const Ipp32s**)values, (const Ipp32s**)levels, nlevels1);
						                  
						                  
				}
				break;
			}
		default :
			// not possible after checkinput
			break;
	}
	return true;
}

