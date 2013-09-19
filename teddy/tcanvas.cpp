#include "tcanvas.h"

#include <QMouseEvent>
#include <QVector3D>
#include <QVector2D>
#include <QtDebug>

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

TCanvas::TCanvas(QWidget *parent)
	: QGLWidget(parent),
	m_fileInfo(tr(UNTITLED)), 
	m_penCursor(QPixmap(":/TMainWind/Resources/other/white/pencil_icon&16.png"), 0, 16),
	m_mouseViewing(true)
{
	m_scene = new TScene(this);

	setAutoFillBackground(false);
	setCursor(m_penCursor);

	m_stepLength = 22.1;
	//m_stepLengthRemained = 0;
}

TCanvas::~TCanvas()
{

}

void TCanvas::initializeGL()
{
	glEnable(GL_MULTISAMPLE);
}

void TCanvas::resizeGL(int w, int h)
{
	m_scene->setupViewport(w, h);
}




void TCanvas::paintEvent(QPaintEvent* e)
{
	makeCurrent();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	qglClearColor(Qt::black);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_POLYGON_SMOOTH);

	m_scene->setupViewport(width(), height());

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	static GLfloat m_diffuse[] = {1.f, 1.f, 1.f, 1.0f};//漫反射光颜色
	static GLfloat m_ambient[] = {1.f, 1.f, 1.f, 1.f};//环境光颜色	
	static GLfloat m_specular[] = {1.f, 1.f, 1.f, 1.f};//镜面反射光颜色
	static GLfloat m_shininess = 1.0f;//镜面指数

	static GLfloat mat_diffuse[] = {0.5f, 0.5f, 0.8f, .5f};//漫反射光颜色
	static GLfloat mat_ambient[] = {0.5f, 0.8f, 0.5f, 0.8f};//环境光颜色	
	static GLfloat mat_specular[] = {0.5f, 0.8f, 0.8f, 0.5f};//镜面反射光颜色
	static GLfloat mat_shininess = 2.0f;//镜面指数

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	static GLfloat lightPosition[4] = { 6.5, 10.0, 14.0, 1.0 };
	static GLfloat lightPosition1[4] = {10, -20, -3, 1};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
	
	m_scene->paint();

	glShadeModel(GL_FLAT);
	//glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_POLYGON_SMOOTH);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();


	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setPen(QPen(Qt::red, 2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
	painter.drawPolyline(m_sketch);
	painter.end();
}

void TCanvas::mousePressEvent(QMouseEvent * e)
{
	m_mouseViewing = true;
	m_mouseLastPos = e->posF();
	if(e->buttons() & Qt::RightButton)
		setCursor(Qt::OpenHandCursor);
	else if(e->buttons() & Qt::MidButton)
		setCursor(Qt::SizeAllCursor);
	else{// left button
		m_mouseViewing = false;
		m_sketch.clear();
		m_sketch.append(m_mouseLastPos);
		setCursor(m_penCursor);
	}
}

void TCanvas::mouseMoveEvent(QMouseEvent * e)
{
	m_mouseViewing = true;
	QVector3D t(e->posF() - m_mouseLastPos);
	double side = qMax(width(), height());
	t.setX( - t.x());
	t /= side / 20.0;
	if(e->buttons() & Qt::RightButton){
		m_scene->camMoveView(t);
		setCursor(Qt::ClosedHandCursor);
		update();
	}else if(e->buttons() & Qt::MidButton){
		m_scene->camMoveCenter(t);
		update();
	}else{
		m_mouseViewing = false;
		/*if(m_sketch.empty()){
			m_sketch.append(e->posF());
			m_stepLengthRemained = 0;
		}else{
			QVector2D newV(e->posF() - m_sketch.last());
			double newDist = newV.length();
			m_stepLengthRemained += newDist;
			while(m_stepLengthRemained > m_stepLength){
				m_stepLengthRemained -= m_stepLength;
				m_sketch.append(e->posF() - newV.normalized().toPointF() * m_stepLengthRemained);
			}		
		}		*/
		m_sketch.append(e->posF());
		update();
	}
	m_mouseLastPos = e->posF();
}

void TCanvas::mouseReleaseEvent(QMouseEvent * e)
{
	setCursor(m_penCursor);
	if(m_mouseViewing)
		return;

	if(m_mode == Creation){
		if(m_sketch.size() >= 2){			
			// close sketch
			//QVector2D v(m_sketch.first() - m_sketch.last());
			//double dist = v.length();
			//while(dist > m_stepLength){
			//	dist -= m_stepLength;
			//	m_sketch.append(m_sketch.first() - v.normalized().toPointF() * dist);
			//}

			if(m_scene->build(m_sketch, m_stepLength)){
				emit creationFinished();
			}
			m_sketch.clear();
		}
	}else if(m_mode == Painting){

	}else if(m_mode == Extrusion){

	}else if(m_mode == Bending){

	}
	update();
}

void TCanvas::wheelEvent(QWheelEvent * e)
{
	m_scene->camZoom(e->delta() / 1000.0);
	update();
}

void TCanvas::neww()
{
	m_scene->deleteLater();
	m_scene = new TScene(this);
	update();
}

void TCanvas::open( const QString& filePath /*= tr("untitled.off")*/ )
{
	if(m_scene->open(filePath)){
		m_fileInfo = QFileInfo(filePath);
		update();
	}
}

void TCanvas::save( const QString& filePath /*= QString()*/ )
{
	if(filePath == QString()){
		m_scene->save(m_fileInfo.absolutePath());
	}else if(m_scene->save(filePath)){
		m_fileInfo = QFileInfo(filePath);
		update();
	}		
}

void TCanvas::setMode( TMode m )
{
	m_mode = m;
}

void TCanvas::meshSmooth()
{
	m_scene->meshSmooth();
	update();
}

void TCanvas::setMeshView( bool useFace )
{
	m_scene->setMeshView(useFace);
	update();
}
