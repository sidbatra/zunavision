/******************************************************************************
** STAIR VISION LIBRARY
** Copyright (C) 2007, Stephen Gould
**
** FILENAME:    svlPoint2d.h
** AUTHOR(S):   Stephen Gould <sgould@stanford.edu>
**              Ian Goodfellow <ia3n@stanford.edu>
**              Paul Baumstarck <pbaumstarck@stanford.edu>
** DESCRIPTION:
**  The svlPoint2d class stripped down to 2D. Pan and tilt are not supported
**  in 2D.
** 
*****************************************************************************/

#pragma once

#if defined(_WIN32)||defined(WIN32)||defined(__WIN32__)
#define _CRT_SECURE_NO_DEPRECATE
#include <limits>
#undef max
#undef min
#endif

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace std;

class svlPoint2d {
 public:
    double x;
    double y;

 public:
    inline svlPoint2d() 
	{
	x = 0.0;
	y = 0.0;
    }
    inline svlPoint2d(double v) 
	{
        x = v; y = v;
    }
    inline svlPoint2d(double nx, double ny) {
      x = nx;
	  y = ny;
    }
    inline svlPoint2d(const svlPoint2d& p) {
	x = p.x; y = p.y;
    }
    virtual ~svlPoint2d() {
	// do nothing
    }

	inline int size();
    inline double getD(int d) const;
    inline void setD(int d, double val);
	inline void set(double val);
	inline void set(double _x, double _y, double _z);
	inline double& operator[](int i);
	inline double operator[](int i) const;

    inline double norm2() const;
	void set_min_pt();
	void set_max_pt();
    inline double dot(const svlPoint2d& p) const;
    inline svlPoint2d& normalize();
	inline double pt_min() const;
    inline double pt_max() const;
    friend inline svlPoint2d min2(const svlPoint2d& p, const svlPoint2d& q);
    friend inline svlPoint2d max2(const svlPoint2d& p, const svlPoint2d& q);

	inline svlPoint2d& min_with(const svlPoint2d& p);
	inline svlPoint2d& min_with(double x_in, double y_in);
    inline svlPoint2d& max_with(const svlPoint2d& p);
	inline svlPoint2d& max_with(double x_in, double y_in);
    
    inline svlPoint2d& roll(double theta);
    inline svlPoint2d& roll(double theta, svlPoint2d& c);

    inline svlPoint2d& operator=(const svlPoint2d& p);
    inline svlPoint2d& operator=(const double d);
    inline bool operator>(const double d);
    inline bool operator>=(const double d);
    inline bool operator>=(const svlPoint2d& p);
    inline bool operator<(const double d);
    inline bool operator<=(const double d);
    inline bool operator<=(const svlPoint2d& p);
    inline svlPoint2d& operator+=(const svlPoint2d& p);
    inline svlPoint2d& operator+=(const double d);
    inline svlPoint2d& operator-=(const svlPoint2d& p);
    inline svlPoint2d& operator-=(const double d);
    inline svlPoint2d& operator*=(const svlPoint2d& p);
    inline svlPoint2d& operator*=(const double d);
    inline svlPoint2d& operator/=(const svlPoint2d& p);
    inline svlPoint2d& operator/=(const double d);
    inline bool operator==(const svlPoint2d& p);
    inline bool operator==(const double d);

    friend inline svlPoint2d operator-(const svlPoint2d& p);
    friend inline svlPoint2d operator+(const svlPoint2d& p, const svlPoint2d& q);
    friend inline svlPoint2d operator+(const svlPoint2d& p, double d);
    friend inline svlPoint2d operator-(const svlPoint2d& p, const svlPoint2d& q);
    friend inline svlPoint2d operator-(const svlPoint2d& p, double d);
    friend inline svlPoint2d operator*(double d, const svlPoint2d& p);
    friend inline svlPoint2d operator*(const svlPoint2d& p, double d);
    friend inline svlPoint2d operator*(const svlPoint2d& p, const svlPoint2d& q);
    friend inline svlPoint2d operator/(const svlPoint2d& p, double d);

    friend std::ostream& operator<<(std::ostream& os, const svlPoint2d& p) {
	os << p.x << " " << p.y << " ";
	return os;
    }
    friend std::istream& operator>>(std::istream& is, svlPoint2d& p) {
	is >> p.x >> p.y;
	return is;
    }
};

/* Implementation ***********************************************************/

inline int svlPoint2d::size() {
	return 2;
}

inline double svlPoint2d::getD(int d)  const
{
    switch(d) {
        case 0: return x; break;
        case 1: return y; break;
        default: 
	assert(false);
	return 0.0;
    }
}

inline void svlPoint2d::setD(int d, double val) 
{
    switch(d) {
        case 0: x = val; break;
        case 1: y = val; break;
        default: 
	assert(false);
    }
}

inline void svlPoint2d::set(double val)
{
	x = y = val;
}

inline void svlPoint2d::set(double _x, double _y, double _z)
{
	x = _x;
	y = _y;
}

inline double& svlPoint2d::operator[](int i)
{
	switch(i) {
		case 0: return this->x;
		case 1: return this->y;
		default: assert(false);
	}
	return this->x;
}

inline double svlPoint2d::operator[](int i) const
{
	switch(i) {
		case 0: return this->x;
		case 1: return this->y;
		default: assert(false);
	}
	return this->x;
}

inline double svlPoint2d::norm2() const
{
    return (x * x + y * y);
}

inline void svlPoint2d::set_min_pt()
{
	x = y = numeric_limits<double>::min();
}

inline void svlPoint2d::set_max_pt()
{
	x = y = numeric_limits<double>::max();
}

inline double svlPoint2d::dot(const svlPoint2d& p) const
{
    return (x * p.x + y * p.y);
}

inline svlPoint2d& svlPoint2d::normalize()
{
    double len = sqrt(norm2());
    x /= len; y /= len;;
    return *this;
}

inline double svlPoint2d::pt_min() const
{
	return x < y ? x : y;
}

inline double svlPoint2d::pt_max() const
{
	return x > y ? x : y;
}

inline svlPoint2d min2(const svlPoint2d& p, const svlPoint2d& q)
{
	return svlPoint2d( p.x < q.x ? p.x : q.x, p.y < q.y ? p.y : q.y);
}

inline svlPoint2d max2(const svlPoint2d& p, const svlPoint2d& q)
{
	return svlPoint2d( p.x > q.x ? p.x : q.x, p.y > q.y ? p.y : q.y);
}

inline svlPoint2d& svlPoint2d::min_with(const svlPoint2d& p) {
	x = x < p.x ? x : p.x;
	y = y < p.y ? y : p.y;
	return *this;
}

inline svlPoint2d& svlPoint2d::min_with(double x_in, double y_in) {
	x = x < x_in ? x : x_in;
	y = y < y_in ? y : y_in;
	return *this;
}

inline svlPoint2d& svlPoint2d::max_with(const svlPoint2d& p) {
	x = x > p.x ? x : p.x;
	y = y > p.y ? y : p.y;
	return *this;
}

inline svlPoint2d& svlPoint2d::max_with(double x_in, double y_in) {
	x = x > x_in ? x : x_in;
	y = y > y_in ? y : y_in;
	return *this;
}


inline svlPoint2d& svlPoint2d::roll(double theta)
{
    double nx = x * cos(theta) - y * sin(theta);
    double ny = x * sin(theta) + y * cos(theta);
    x = nx; y = ny;

    return *this;
}

inline svlPoint2d& svlPoint2d::roll(double theta, svlPoint2d& c)
{
    double nx = (x - c.x) * cos(theta) - (y - c.y) * sin(theta);
    double ny = (x - c.x) * sin(theta) + (y - c.y) * cos(theta);
    x = c.x + nx; y = c.y + ny;    

    return *this;
}

/* Operators ****************************************************************/

inline svlPoint2d& svlPoint2d::operator=(const svlPoint2d& p)
{
  x = p.x; y = p.y;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator=(const double d)
{
    x = y  = d;
    return *this;
}

inline bool svlPoint2d::operator>(const double d)
{
    return ((x > d) && (y > d));
}

inline bool svlPoint2d::operator>=(const double d)
{
    return ((x >= d) && (y >= d));
}

inline bool svlPoint2d::operator>=(const svlPoint2d& p)
{
  return ((x >= p.x) && (y >= p.y));
}

inline bool svlPoint2d::operator<(const double d)
{
  return ((x < d) && (y < d));
}

inline bool svlPoint2d::operator<=(const double d)
{
  return ((x <= d) && (y <= d));
}

inline bool svlPoint2d::operator<=(const svlPoint2d& p)
{
  return ((x <= p.x) && (y <= p.y));
}

inline svlPoint2d& svlPoint2d::operator+=(const svlPoint2d& p)
{
    x += p.x; y += p.y;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator+=(const double d)
{
    x += d; y += d;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator-=(const svlPoint2d& p)
{
    x -= p.x; y -= p.y;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator-=(const double d)
{
    x -= d; y -= d;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator*=(const svlPoint2d& p)
{
    x *= p.x; y *= p.y;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator*=(const double d)
{
    x *= d; y *= d;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator/=(const svlPoint2d& p)
{
    x /= p.x; y /= p.y;
    return *this;
}

inline svlPoint2d& svlPoint2d::operator/=(const double d)
{
    x /= d; y /= d;
    return *this;
}

inline bool svlPoint2d::operator==(const svlPoint2d& p)
{
    return ((x == p.x) && (y == p.y));	
}

inline bool svlPoint2d::operator==(const double d)
{
    return ((x == d) && (y == d));
}

inline svlPoint2d operator-(const svlPoint2d& p)
{
    return svlPoint2d(-p.x, -p.y);
}

inline svlPoint2d operator+(const svlPoint2d& p, const svlPoint2d& q)
{
    return svlPoint2d(p.x + q.x, p.y + q.y);
}

inline svlPoint2d operator+(const svlPoint2d& p, double d)
{
    return svlPoint2d(p.x + d, p.y + d);
}

inline svlPoint2d operator-(const svlPoint2d& p, const svlPoint2d& q)
{
    return svlPoint2d(p.x - q.x, p.y - q.y);
}

inline svlPoint2d operator-(const svlPoint2d& p, double d)
{
    return svlPoint2d(p.x - d, p.y - d);
}

inline svlPoint2d operator*(double d, const svlPoint2d& p)
{
    return svlPoint2d(d * p.x, d * p.y);
}

inline svlPoint2d operator*(const svlPoint2d& p, double d)
{
    return svlPoint2d(d * p.x, d * p.y);
}

inline svlPoint2d operator*(const svlPoint2d& p, const svlPoint2d& q)
{
    return svlPoint2d(p.x * q.x, p.y * q.y);
}

inline svlPoint2d operator/(const svlPoint2d& p, double d)
{
    return svlPoint2d(p.x / d, p.y / d);
}
