#ifndef IPL_CENTER_DEPENDANT_MODULE_H
#define IPL_CENTER_DEPENDANT_MODULE_H


class IPLCenterDependantModule
{
	protected:
		double cx,cy;
		virtual bool updateImageCenter() = 0;
	public:
		IPLCenterDependantModule() {
			cx = cy = 0;
		}
		virtual ~IPLCenterDependantModule() {}

		bool setImageCenter(double x, double y) {
			if ((x!=cx)||(y!=cy)) {
				cx = x;
				cy = y;
				return this->updateImageCenter();
			}
			return true;
		}
};




#endif // IPL_CENTER_DEPENDANT_MODULE_H
