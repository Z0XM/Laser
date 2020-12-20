#pragma once
#include <iostream>
#include <SFML/Graphics.hpp>

class Lines {
public:
	float m, c;
	sf::Vector2f a, b;

	Lines();
	Lines(sf::Vector2f, sf::Vector2f);
	~Lines();

	float x(float y);
	float y(float x);
	float getValue(sf::Vector2f);

	friend sf::Vector2f intersection(Lines, Lines);
	friend float angle(Lines, Lines);
	friend bool areParallel(Lines, Lines);
	friend std::ostream& operator<<(std::ostream&, Lines);
};

sf::Vector2f intersection(Lines, Lines);
bool areParallel(Lines, Lines);
float Linesangle(Lines, Lines); 
Lines Perpendicular(Lines, sf::Vector2f);
std::ostream& operator<<(std::ostream&, Lines);