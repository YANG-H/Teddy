#include "tmainwind.h"
#include <QtGui>
#include <QSettings>


#include "qtwin.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	TMainWind window;

#ifdef Q_WS_X11
	window.setAttribute(Qt::WA_TranslucentBackground);
	window.setAttribute(Qt::WA_NoSystemBackground, false);
	QPalette pal = window.palette();
	QColor bg = pal.window().color();
	bg.setAlpha(180);
	pal.setColor(QPalette::Window, bg);
	window.setPalette(pal);
	window.ensurePolished(); // workaround Oxygen filling the background
	window.setAttribute(Qt::WA_StyledBackground, false);
#endif
	if (QtWin::isCompositionEnabled()) {
		QtWin::extendFrameIntoClientArea(&window);
		window.setContentsMargins(0, 0, 0, 0);
	}
	window.show();
	return a.exec();
}
