#ifndef TMAINWIND_H
#define TMAINWIND_H

#include <QtGui>
#include "ui_tmainwind.h"

class TCanvas;

class TMainWind : public QMainWindow
{
	Q_OBJECT

public:
	TMainWind(QWidget *parent = 0);
	~TMainWind();

	void init();

public slots:
	void on_actionNew_triggered();
	void on_actionOpen_triggered();
	void on_actionSave_triggered();
	void on_actionSaveAs_triggered();
	
	void on_actionInfo_triggered();

private:
	TCanvas* m_canvas;

	Ui::TMainWindClass ui;
};

#endif // TMAINWIND_H
