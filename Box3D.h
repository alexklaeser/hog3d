#ifndef BOX3D_H_
#define BOX3D_H_

struct Box3D
{
	double x, y, t;
	double width, height, length;
	
	Box3D(double x_ = 0, double y_ = 0, double t_ = 0, double width_ = 0, double height_ = 0, double length_ = 0)
		: x(x_), y(y_), t(t_), width(width_), height(height_), length(length_)
	{ }
	
	virtual ~Box3D()
	{ }
};

#endif /*BOX3D_H_*/
