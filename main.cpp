
#include <Interface.h>

int
main(int argc, char** argv)
{
	Interface::MenuOption currentSelection = Interface::MenuOption::Calibrate;
	Interface interface;
	while (currentSelection != Interface::MenuOption::Quit) {
		currentSelection = interface.getCommand();
		std::cout << currentSelection << "\n";

	}

}