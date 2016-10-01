#ifndef __SSL_HPP__
#define __SSL_HPP__

class SSL
{
private:
	int x_, y_;
	short value_;
	unsigned int scale_;
	
public:
	SSL():x_(0),y_(0),value_(0), scale_(0) {}
	SSL(int x, int y,
		short value=0, unsigned int scale=0):
		x_(x),y_(y),value_(value), scale_(scale) {}
	SSL(const SSL& ssl)
		:x_(ssl.x_)
		, y_(ssl.y_)
		,value_(ssl.value_)
		,scale_(ssl.scale_)
		{}
	
	inline const int X() const {return x_;}
	inline const int Y() const {return y_;}
	inline const short Value() const {return value_;}
	inline const unsigned int Scale() const{ return scale_;}
	
	inline int& X() {return x_;}
	inline int& Y() {return y_;}
	inline short& Value() {return value_;}
	inline unsigned int& Scale() { return scale_;}

	inline unsigned int dist2(const SSL& ssl)
		{ return (x_-ssl.x_)*(x_-ssl.x_)+(x_-ssl.x_)*(x_-ssl.x_); }

	inline void operator=(const SSL& ssl){
		x_=ssl.x_;
		y_=ssl.y_;
		value_=ssl.value_;
		scale_=ssl.scale_;
	}
	
	inline bool operator<(const SSL& ssl) const {
		return value_>ssl.value_;
	}
	
	static bool isLarger( const SSL& ssl1, const SSL& ssl2 ) {
		return ssl1.scale_ > ssl2.scale_;
	}
};

#endif
