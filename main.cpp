#include <iostream>
#include "src/SFML_Control.hpp"

int main()
{
	SFML_Control sfml;

	while (sfml.isRunning())
	{
		sfml.update();
		sfml.render();
	}
}