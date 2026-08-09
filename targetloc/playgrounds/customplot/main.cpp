#include "window.h"
#include <QApplication>

// Main program
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	// create the window
	Window window;
	window.setWindowState(Qt::WindowMaximized);
	window.show();

	const int r = app.exec();

	return r;
}
