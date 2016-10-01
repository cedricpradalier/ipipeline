#ifndef _IPL_STRUCTS_H
#define _IPL_STRUCTS_H

struct IPLPoint3D {

	double range;
	double height;
	double angle;

	double x;
	double y;
	double z;

};


struct IPLObstacle {
	/* Image coordinates of the bottom left hand corner of a box 
	   containing the obstacle. */
	unsigned int disparity;
	int xmin,xmax;
	int ymin,ymax;

	//int x_centre;
	//int y_centre;

	int intensity; // intensity of object where it 
	// meets the ground plane
	//int intensity_top;

	int width;
	int height;

	double x3D;
	double y3D;
	double z3D;

	//double range;
	//double angle;

	//int num_frames; // number of frames object present
	//int num_gone; // number of frames object has disappeared for.

	//bool display;


};

struct IPLLine {
	int x1;
	int y1;

	int x2;
	int y2;

};




#endif // _IPL_STRUCTS_H
