#include "Lines.hpp"
#include <cmath>
#include <iostream>

Lines::Lines() {
	m = 0; c = 0;
}

Lines::Lines(sf::Vector2f a, sf::Vector2f b)
{
	this->a = a;
	this->b = b;
	m = (b.y - a.y) / (b.x - a.x);
	c = a.y - a.x * m;
}

Lines::~Lines()
{
}
float Lines::x(float y)
{
	return (y - c) / m;
}

float Lines::y(float x)
{
	return m * x + c;
}

float Lines::getValue(sf::Vector2f point)
{
	return point.y - m * point.x - c;
}

sf::Vector2f intersection(Lines l, Lines m)
{
	float x = 0, y = 0;
	if (l.m == 0) {
		if (std::isinf(m.m)) {
			y = l.a.y;
			x = m.a.x;
		}
		else if(m.m != 0){
			y = l.a.y;
			x = m.x(y);
		}
	}
	else if (std::isinf(l.m)) {
		if (m.m == 0) {
			y = m.a.y;
			x = l.a.x;
		}
		else if (m.m != 0) {
			x = l.a.x;
			y = m.y(x);
		}
	}
	else {
		if (std::isinf(m.m)) {
			x = m.a.x;
			y = l.y(x);
		}
		else if (m.m == 0) {
			y = m.a.y;
			x = l.x(y);
		}
		else {
			x = (l.c - m.c) / (m.m - l.m);
			y = l.y(x);
		}
	}

	return sf::Vector2f(x, y);
}

bool areParallel(Lines l, Lines n)
{
	return l.m == n.m;
}

float angle(Lines l, Lines m)
{
	return std::abs(atan2f((l.m - m.m), 1 + l.m * m.m));
}

std::ostream& operator<<(std::ostream& out, Lines l)
{
	out << "y = " << l.m << "x + " << l.c << "\n";
	return out;
}