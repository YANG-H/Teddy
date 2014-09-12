#ifndef TCANVAS_H
#define TCANVAS_H

#include <QGLWidget>
#include <QMatrix4x4>
#include <QFileInfo>
#include <QPolygonF>

#include "tscene.h"

#define UNTITLED "untitled.off"

class TCanvas : public QGLWidget
{
	Q_OBJECT
	Q_PROPERTY(TMode mode READ mode WRITE setMode)
	Q_ENUMS(TMode)

public:
	enum TMode {Creation, Bending, Painting, Extrusion};
	TCanvas(QWidget *parent = 0);
	~TCanvas();

	void setMode(TMode m);
	inline TMode mode() const {return m_mode;}

	inline void setFileInfo(const QFileInfo& fi) {m_fileInfo = fi;}
	inline QFileInfo fileInfo() const {return m_fileInfo;}
	inline bool fileExists() const {return m_fileInfo.exists();}

	void neww();
	void open(const QString& filePath = tr(UNTITLED));
	void save(const QString& filePath = QString());

signals:
	void restart();
	void creationFinished();
	void bendingFinished();
	void toEdit();
	void extrusionFinished();

public slots:
	void meshSmooth();
	void setMeshView(bool useFace);

protected:
	void initializeGL();
	void resizeGL(int w, int h);

	void paintEvent(QPaintEvent* e);
	void mousePressEvent(QMouseEvent * e);
	void mouseMoveEvent(QMouseEvent * e);
	void mouseReleaseEvent(QMouseEvent * e);
	void wheelEvent(QWheelEvent * e);

private:
	TMode m_mode;
	TScene* m_scene;
	QPolygonF m_sketch;

	QFileInfo m_fileInfo;

	QPointF m_mouseLastPos;
	QCursor m_penCursor;

	bool m_mouseViewing;

	double m_stepLength;
};

#endif // TCANVAS_H
