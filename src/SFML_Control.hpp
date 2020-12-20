#pragma once
#include <SFML/Graphics.hpp>

class SFML_Control {
private:
	sf::RenderWindow window;

	void InitWindow();
public:
	SFML_Control();
	~SFML_Control();

	bool isRunning();

	void update();
	void pollEvents();
	void render();

	sf::Vector2f getMousePosition();
};