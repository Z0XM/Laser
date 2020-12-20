#include "SFML_Control.hpp"
#include "Laser.hpp"
#include "Reflective.hpp"
#include <iostream>

void SFML_Control::InitWindow()
{
	window.create(sf::VideoMode(1200, 700), "Laser");
}

SFML_Control::SFML_Control()
{
	this->InitWindow();
	
	Reflective::ReflectiveGroup.push_back(new Reflective(sf::Vector2f(0, 0), sf::Vector2f(window.getSize().x, 0)));
	Reflective::ReflectiveGroup.push_back(new Reflective(sf::Vector2f(window.getSize().x, window.getSize().y), sf::Vector2f(window.getSize().x, 0)));
	Reflective::ReflectiveGroup.push_back(new Reflective(sf::Vector2f(window.getSize().x, window.getSize().y), sf::Vector2f(0, window.getSize().y)));
	Reflective::ReflectiveGroup.push_back(new Reflective(sf::Vector2f(0, 0), sf::Vector2f(0, window.getSize().y)));
}

SFML_Control::~SFML_Control()
{
	LaserControl::delGroup();
	Reflective::delGroup();
}

bool SFML_Control::isRunning()
{
	return this->window.isOpen();
}

void SFML_Control::update()
{
	this->pollEvents();
}

void SFML_Control::pollEvents()
{
	sf::Event event;
	sf::Vector2f oldPos = this->getMousePosition();
	while (this->window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed)
			this->window.close();

		if (event.type == sf::Event::MouseButtonPressed) {
			LaserControl::setSelectedLaser(oldPos);
			Reflective::setSelectedReflector(oldPos);
		}
		else if (event.type == sf::Event::MouseButtonReleased) {
			LaserControl::resetSelection();
			Reflective::resetSelection();
		}

		if (event.type == sf::Event::KeyPressed) {
			if (event.key.code == sf::Keyboard::L) {
				LaserControl::LaserGroup.push_back(new LaserControl(oldPos));
			}
			else if (event.key.code == sf::Keyboard::O) {
				LaserControl::LaserGroup.push_back(new LaserControl((sf::Vector2f)(window.getSize()) * 0.5f));
			}
			else if (event.key.code == sf::Keyboard::R) {
				Reflective::ReflectiveGroup.push_back(new Reflective({ oldPos.x - 50, oldPos.y }, { oldPos.x + 50, oldPos.y }));
			}
			else if (event.key.code == sf::Keyboard::Z) {
				if(LaserControl::LaserMode == LaserControl::NORMAL)LaserControl::LaserMode = LaserControl::ANIMATION;
				else if(LaserControl::LaserMode == LaserControl::ANIMATION)LaserControl::LaserMode = LaserControl::NORMAL;
			}
			else if (event.key.code == sf::Keyboard::Up) {
				LaserControl::nextReflections();
			}
			else if (event.key.code == sf::Keyboard::Down) {
				LaserControl::prevReflections();
			}
		}
	}

	if (sf::Mouse::isButtonPressed(sf::Mouse::Left)){
		if(LaserControl::ActiveSelection != nullptr)LaserControl::mouseMove(this->getMousePosition());
		if(Reflective::ActiveSelection != nullptr)Reflective::mouseMove(this->getMousePosition());
	}
	if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
		if (LaserControl::ActiveSelection != nullptr)LaserControl::rotate();
		if (Reflective::ActiveSelection != nullptr)Reflective::rotate();
	}
}

void SFML_Control::render()
{
	if(LaserControl::LaserMode == LaserControl::NORMAL)this->window.clear();

	LaserControl::drawGroup(this->window);
	if (LaserControl::LaserMode == LaserControl::NORMAL)Reflective::drawGroup(this->window);

	if (LaserControl::LaserMode == LaserControl::NORMAL)this->window.display();
}

sf::Vector2f SFML_Control::getMousePosition()
{
	return window.mapPixelToCoords(sf::Mouse::getPosition(this->window));
}