#include "Reflective.hpp"

#include <iostream>

std::vector<Reflective*> Reflective::ReflectiveGroup;
Reflective* Reflective::ActiveSelection = nullptr;
unsigned Reflective::selectedShape = 0;

Reflective::Reflective(sf::Vector2f a, sf::Vector2f b)
{
	this->vertex[0].setFillColor(sf::Color::Red);
	this->vertex[0].setRadius(7);
	this->vertex[0].setOrigin(this->vertex[0].getRadius(), this->vertex[0].getRadius());
	this->vertex[0].setPosition(a);

	this->vertex[1].setFillColor(sf::Color::Red);
	this->vertex[1].setRadius(7);
	this->vertex[1].setOrigin(this->vertex[0].getRadius(), this->vertex[0].getRadius());
	this->vertex[1].setPosition(b);

	this->vertex[2].setFillColor(sf::Color::Blue);
	this->vertex[2].setRadius(5);
	this->vertex[2].setOrigin(this->vertex[2].getRadius(), this->vertex[2].getRadius());
	this->vertex[2].setPosition((a.x + b.x) / 2, (a.y + b.y) / 2);

	this->rotateAngle = 0.0005;

	surface = Lines(a, b);
}

Reflective::~Reflective()
{
}

void Reflective::draw(sf::RenderWindow& window)
{
	window.draw(this->vertex[0]);
	window.draw(this->vertex[1]);
	window.draw(this->vertex[2]);

	sf::Vertex vertices[2] = { sf::Vertex(vertex[0].getPosition(), sf::Color::Blue), sf::Vertex(vertex[1].getPosition(), sf::Color::Blue) };
	window.draw(vertices, 2, sf::Lines);
}

void Reflective::drawGroup(sf::RenderWindow& window)
{
	for (auto reflector : ReflectiveGroup)
		reflector->draw(window);
}

void Reflective::delGroup()
{
	auto it = ReflectiveGroup.begin();
	while (it != ReflectiveGroup.end())
		it = ReflectiveGroup.erase(it);
}

void Reflective::setSelectedReflector(sf::Vector2f position)
{
	for (int i = 0; i < ReflectiveGroup.size(); i++) {
		if (ReflectiveGroup[i]->vertex[0].getGlobalBounds().contains(position)) {
			ActiveSelection = ReflectiveGroup[i];
			selectedShape = 1;
			break;
		}
		else if (ReflectiveGroup[i]->vertex[1].getGlobalBounds().contains(position)) {
			ActiveSelection = ReflectiveGroup[i];
			selectedShape = 2;
			break;
		}
		else if (ReflectiveGroup[i]->vertex[2].getGlobalBounds().contains(position)) {
			ActiveSelection = ReflectiveGroup[i];
			selectedShape = 3;
			break;
		}
	}
}

void Reflective::resetSelection()
{
	if (ActiveSelection != nullptr)ActiveSelection->rotateAngle = 0.0005;
	ActiveSelection = nullptr;
	selectedShape = 0;
}

sf::Vector2f Reflective::getPosition()
{
	return this->vertex[0].getPosition();
}

void Reflective::mouseMove(sf::Vector2f pos)
{
	if (selectedShape == 1) {
		ActiveSelection->vertex[0].setPosition(pos);
		sf::Vector2f a = ActiveSelection->vertex[0].getPosition(), b = ActiveSelection->vertex[1].getPosition();
		ActiveSelection->vertex[2].setPosition((a.x + b.x) / 2, (a.y + b.y) / 2);
	}
	if (selectedShape == 2) {
		ActiveSelection->vertex[1].setPosition(pos);
		sf::Vector2f a = ActiveSelection->vertex[0].getPosition(), b = ActiveSelection->vertex[1].getPosition();
		ActiveSelection->vertex[2].setPosition((a.x + b.x) / 2, (a.y + b.y) / 2);
	}
	if (selectedShape == 3) {
		sf::Vector2f c = ActiveSelection->vertex[2].getPosition();
		ActiveSelection->vertex[0].setPosition(ActiveSelection->vertex[0].getPosition() - c + pos);
		ActiveSelection->vertex[1].setPosition(ActiveSelection->vertex[1].getPosition() - c + pos);
		ActiveSelection->vertex[2].setPosition(pos);
	}
	ActiveSelection->surface = Lines(ActiveSelection->vertex[0].getPosition(), ActiveSelection->vertex[1].getPosition());
}

void Reflective::rotate()
{
	sf::Vector2f a = ActiveSelection->vertex[0].getPosition(), b = ActiveSelection->vertex[1].getPosition(), c = ActiveSelection->vertex[2].getPosition();

	float angle = ActiveSelection->rotateAngle;

	if (selectedShape == 1) {
		angle += atan2f(b.y - a.y, b.x - a.x);
		float d = sqrt((b.y - a.y) * (b.y - a.y) + (b.x - a.x) * (b.x - a.x));
		ActiveSelection->vertex[1].setPosition(a + sf::Vector2f(d * cos(angle), d * sin(angle)));
		ActiveSelection->vertex[2].setPosition(a + sf::Vector2f((d / 2) * cos(angle), (d / 2) * sin(angle)));
	}
	else if (selectedShape == 2) {
		angle += atan2f(a.y - b.y, a.x - b.x);
		float d = sqrt((b.y - a.y) * (b.y - a.y) + (b.x - a.x) * (b.x - a.x));
		ActiveSelection->vertex[0].setPosition(b + sf::Vector2f(d * cos(angle), d * sin(angle)));
		ActiveSelection->vertex[2].setPosition(b + sf::Vector2f((d / 2) * cos(angle), (d / 2) * sin(angle)));
	}
	else if (selectedShape == 3) {
		angle += atan2f(a.y - c.y, a.x - c.x);
		float d = sqrt((b.y - c.y) * (b.y - c.y) + (b.x - c.x) * (b.x - c.x));
		ActiveSelection->vertex[0].setPosition(c + sf::Vector2f(d * cos(angle), d * sin(angle)));
		angle += atan2f(b.y - c.y, b.x - c.x) - atan2f(a.y - c.y, a.x - c.x);
		ActiveSelection->vertex[1].setPosition(c + sf::Vector2f(d * cos(angle), d * sin(angle)));
	}
	ActiveSelection->surface = Lines(ActiveSelection->vertex[0].getPosition(), ActiveSelection->vertex[1].getPosition());
	ActiveSelection->rotateAngle += 0.000001;
}