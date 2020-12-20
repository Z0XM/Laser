#include "Laser.hpp"
#include "Reflective.hpp"
#include <iostream>

std::vector<LaserControl*> LaserControl::LaserGroup;
LaserControl* LaserControl::ActiveSelection = nullptr;
unsigned LaserControl::selectedShape = 0;
float LaserControl::RotateConstant = 0.0002;
unsigned int LaserControl::ReflectionConstant = 500;
unsigned int LaserControl::Reflections[11] = { 1, 2, 3, 5, 10, 50, 100, 500, 1000, 5000, 10000 };
LaserControl::Modes LaserControl::LaserMode = LaserControl::NORMAL;
int LaserControl::reflectionCounter = 0;

LaserControl::LaserControl(sf::Vector2f position)
{
	this->origin.setFillColor(sf::Color::White);
	this->origin.setRadius(7);
	this->origin.setOrigin(this->origin.getRadius(), this->origin.getRadius());
	this->origin.setPosition(position);

	this->face.setFillColor(sf::Color::Green);
	this->face.setRadius(7);
	this->face.setOrigin(this->origin.getRadius(), this->origin.getRadius());
	this->face.setPosition(position.x + 30, position.y);

	this->rotateAngle = RotateConstant;
}

LaserControl::~LaserControl()
{
}

void LaserControl::draw(sf::RenderWindow& window)
{
	window.draw(this->origin);
	window.draw(this->face);
}

void LaserControl::drawGroup(sf::RenderWindow& window)
{
	if (LaserMode == NORMAL) {
		for (auto laser : LaserGroup) {
			laser->draw(window);
			laser->drawLaser(window);
		}
	}
	else if (LaserMode == ANIMATION) {
		for (auto laser : LaserGroup) {
			laser->drawLaser(window);
		}
	}
}

void LaserControl::delGroup()
{
	auto it = LaserGroup.begin();
	while (it != LaserGroup.end())
		it = LaserGroup.erase(it);
}

void LaserControl::setSelectedLaser(sf::Vector2f position)
{
	for (int i = 0; i < LaserGroup.size(); i++) {
		if (LaserGroup[i]->origin.getGlobalBounds().contains(position)) {
			ActiveSelection = LaserGroup[i];
			selectedShape = 1;
			break;
		}
		else if (LaserGroup[i]->face.getGlobalBounds().contains(position)) {
			ActiveSelection = LaserGroup[i];
			selectedShape = 2;
			break;
		}
	}
}

void LaserControl::resetSelection()
{
	if(ActiveSelection != nullptr)ActiveSelection->rotateAngle = RotateConstant;
	ActiveSelection = nullptr;
	selectedShape = 0;
}

sf::Vector2f LaserControl::getPosition()
{
	return this->origin.getPosition();
}

void LaserControl::mouseMove(sf::Vector2f pos)
{
	if (selectedShape == 1) {
		ActiveSelection->face.setPosition(pos - ActiveSelection->origin.getPosition() + ActiveSelection->face.getPosition());
		ActiveSelection->origin.setPosition(pos);
	}
	if (selectedShape == 2) {
		auto angle = atan2f(pos.y - ActiveSelection->origin.getPosition().y, pos.x - ActiveSelection->origin.getPosition().x);
		ActiveSelection->face.setPosition(ActiveSelection->origin.getPosition() + sf::Vector2f(30 * cos(angle), 30 * sin(angle)));
	}
}

void LaserControl::rotate()
{
	float angle = ActiveSelection->rotateAngle;
	if (selectedShape == 1) {
		angle += atan2f(ActiveSelection->face.getPosition().y - ActiveSelection->origin.getPosition().y, ActiveSelection->face.getPosition().x - ActiveSelection->origin.getPosition().x);
		ActiveSelection->face.setPosition(ActiveSelection->origin.getPosition() + sf::Vector2f(30 * cos(angle), 30 * sin(angle)));
	}
	else if (selectedShape == 2) {
		angle += atan2f(ActiveSelection->origin.getPosition().y - ActiveSelection->face.getPosition().y, ActiveSelection->origin.getPosition().x - ActiveSelection->face.getPosition().x);
		ActiveSelection->origin.setPosition(ActiveSelection->face.getPosition() + sf::Vector2f(30 * cos(angle), 30 * sin(angle)));
	}
	//ActiveSelection->rotateAngle += 0.000001;
}

void LaserControl::nextReflections()
{
	reflectionCounter++;
	if (reflectionCounter > 10)reflectionCounter = 0;
	ReflectionConstant = Reflections[reflectionCounter];
}

void LaserControl::prevReflections()
{
	reflectionCounter--;
	if (reflectionCounter < 0)reflectionCounter = 10;
	ReflectionConstant = Reflections[reflectionCounter];
}

std::pair<Lines, Reflective*> LaserControl::findReflector(Lines laser, Reflective* skipReflector)
{
	auto pointAlignment = [](sf::Vector2f a, sf::Vector2f b, sf::Vector2f c) {
		sf::Vector2f ab = a - b, cb = c - b;
		if ((ab.x > 0 && cb.x > 0) || (ab.x < 0 && cb.x < 0) || (ab.y > 0 && cb.y > 0) || (ab.y < 0 && cb.y < 0))return false;
		return true;
	};

	sf::Vector2f intersectionPoint, o = laser.a, f = laser.b;
	double laserLength = 0;
	Lines reflector;
	Reflective* localSkipReflector;
	int g = 0;
	for (auto it = Reflective::ReflectiveGroup.begin(); it != Reflective::ReflectiveGroup.begin() + 4; it++, g++) {
		if (*it == skipReflector)continue; 
		
		Lines localReflector = (*it)->surface;
		if (areParallel(laser, localReflector))continue;

		sf::Vector2f k = intersection(laser, localReflector);
		if (!pointAlignment(o, f, k) || !pointAlignment(localReflector.a, k, localReflector.b))continue;

		intersectionPoint = k;
		reflector = localReflector;
		localSkipReflector = *it;
		laserLength = std::sqrt((k.y - o.y) * (k.y - o.y) + (k.x - o.x) * (k.x - o.x));
		
		break;
	}

	for (auto it = Reflective::ReflectiveGroup.begin() + 4; it != Reflective::ReflectiveGroup.end(); it++) {
		if (*it == skipReflector)continue;

		Lines localReflector = (*it)->surface;
		if (areParallel(laser, localReflector))continue;

		sf::Vector2f k = intersection(laser, localReflector);
		if (!pointAlignment(o, f, k) || !pointAlignment(localReflector.a, k, localReflector.b))continue;

		double d = std::sqrt((k.y - o.y) * (k.y - o.y) + (k.x - o.x) * (k.x - o.x));
		if (d < laserLength) {
			intersectionPoint = k;
			reflector = localReflector;
			localSkipReflector = *it;
			laserLength = d;
		}
	}

	//perpendicular to reflector
	float m = -1 / reflector.m, z;
	sf::Vector2f pointBehind;

	if (std::isinf(m)) {
		z = -2 * (laser.b.x - intersectionPoint.x);
		pointBehind = sf::Vector2f(z + laser.b.x, laser.b.y) - intersectionPoint;
	}
	else {
		z = -2 * ((laser.b.y - m * laser.b.x - intersectionPoint.y + m * intersectionPoint.x) / (m * m + 1));
		pointBehind = sf::Vector2f(z * -m + laser.b.x, z + laser.b.y) - intersectionPoint;
	}

	float mag = std::sqrt(pointBehind.x * pointBehind.x + pointBehind.y * pointBehind.y);
	
	pointBehind = sf::Vector2f(-30 * (pointBehind.x / mag), -30 * (pointBehind.y / mag));

	return std::make_pair(Lines(pointBehind + intersectionPoint, intersectionPoint), localSkipReflector);
}

void LaserControl::drawLaser(sf::RenderWindow& window)
{
	sf::Vector2f o = origin.getPosition(), f = face.getPosition();

	sf::Vertex laser[2];
	laser[0].color = sf::Color::Green;
	laser[1].color = sf::Color::Green;

	if (LaserMode == NORMAL) {
		laser[0].position = o;
		auto data = this->findReflector(Lines(o, f), nullptr);
		laser[1].position = data.first.b;
		window.draw(laser, 2, sf::Lines);

		for (int i = 0; i < ReflectionConstant; i++) {
			laser[0].position = data.first.b;
			data = this->findReflector(data.first, data.second);
			laser[1].position = data.first.b;
			window.draw(laser, 2, sf::Lines);
		}
	}
	else if (LaserMode == ANIMATION) {
		laser[0].position = o;
		auto data = this->findReflector(Lines(o, f), nullptr);
		laser[1].position = data.first.b;
		sf::Vector2f p = laser[1].position - laser[0].position;
		float mag = std::sqrt(p.x * p.x + p.y * p.y);
		p = sf::Vector2f(p.x / mag, p.y / mag);
		float length = 0;
		window.clear();
		Reflective::drawGroup(window);
		this->draw(window);
		while (length <= mag) {
			window.draw(&laser[0], 1 , sf::Points);
			window.display();
			laser[0].position += p;
			length++;
		}
		std::cout << "\nnew";
		for (int i = 0; i < ReflectionConstant; i++) {
			std::cout << i <<"\n";
			laser[0].position = data.first.b;
			data = this->findReflector(data.first, data.second);
			laser[1].position = data.first.b;
			sf::Vector2f p = laser[1].position - laser[0].position;
			float mag = std::sqrt(p.x * p.x + p.y * p.y);
			p = sf::Vector2f(p.x / mag, p.y / mag);
			float length = 0;
			window.clear();
			Reflective::drawGroup(window);
			this->draw(window);
			while (length <= mag) {
				window.draw(&laser[0], 1, sf::Points);
				window.display();
				laser[0].position += p;
				length++;
			}
		}
	}
}
