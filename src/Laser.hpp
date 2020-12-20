#pragma once

#include "Reflective.hpp"
#include "Lines.hpp"
#include <SFML/Graphics.hpp>
#include <vector>
#include <algorithm>

class LaserControl {
private:
	sf::CircleShape origin, face;
	float rotateAngle;
public:
	LaserControl(sf::Vector2f);
	~LaserControl();

	void draw(sf::RenderWindow&);
	static void drawGroup(sf::RenderWindow&);
	static void delGroup();
	static void setSelectedLaser(sf::Vector2f);
	static void resetSelection();
	static void mouseMove(sf::Vector2f);
	static void rotate();
	static void nextReflections();
	static void prevReflections();

	sf::Vector2f getPosition();
	void drawLaser(sf::RenderWindow&);
	std::pair<Lines, Reflective*> findReflector(Lines, Reflective*);
	
	enum Modes{NORMAL, ANIMATION};
	static Modes LaserMode;
	static unsigned int ReflectionConstant;
	static unsigned int Reflections[11];
	static float RotateConstant;
	static unsigned selectedShape;
	static std::vector<LaserControl*> LaserGroup;
	static LaserControl* ActiveSelection;
	static int reflectionCounter;
};