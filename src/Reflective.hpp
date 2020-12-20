#pragma once

#include <SFML/Graphics.hpp>
#include "Lines.hpp"

class Reflective {
private:
	sf::CircleShape vertex[3];
	float rotateAngle;

public:
	Reflective(sf::Vector2f, sf::Vector2f);
	~Reflective();

	void draw(sf::RenderWindow&);
	static void drawGroup(sf::RenderWindow&);
	static void delGroup();
	static void setSelectedReflector(sf::Vector2f);
	static void resetSelection();
	static void mouseMove(sf::Vector2f);
	static void rotate();

	sf::Vector2f getPosition();

	Lines surface;
	static unsigned selectedShape;
	static std::vector<Reflective*> ReflectiveGroup;
	static Reflective* ActiveSelection;
};