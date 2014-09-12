#include "tscene.h"

#include <vector>

#include <QString>
#include <QMatrix3x3>
#include <QtOpenGL>

#include <gl/GLU.h>

//#include <fstream>
//
//#include <CGAL/IO/Polyhedron_iostream.h>
//#include <CGAL/IO/Polyhedron_inventor_ostream.h>
//#include <CGAL/IO/Polyhedron_VRML_1_ostream.h>
//#include <CGAL/IO/Polyhedron_VRML_2_ostream.h>
//#include <CGAL/IO/Polyhedron_geomview_ostream.h>
//
//#include <list>
//
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Constrained_Delaunay_triangulation_2.h>
//#include <CGAL/Triangulation_conformer_2.h>
//
//#include <iostream>
//
//typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
//typedef CGAL::Constrained_Delaunay_triangulation_2<K> CDT;

inline void qglVertex3d(const QVector3D& v){glVertex3d(v.x(), v.y(), v.z());}
//inline void cgglVertex3d(const Point_3& p){glVertex3d(p.x(), p.y(), p.z());}
inline void qglNormal3d(const QVector3D& v){glNormal3d(v.x(), v.y(), v.z());}
//inline void cgglNormal3d(const Vector_3& d){glNormal3d(d.x(), d.y(), d.z());}

TScene::TScene(QObject *parent /* = 0 */)
	: QObject(parent), m_faceView(false)
{
	m_mesh.request_vertex_normals();
	Q_ASSERT(m_mesh.has_vertex_normals());

	m_cam_eye = QVector3D(0, 0, 2.0);
	m_cam_center = QVector3D(0, 0, 0);
	m_cam_up = QVector3D(0, 1, 0);

	m_cam.lookAt(m_cam_eye, m_cam_center, m_cam_up);
	m_proj.perspective(45, 1.0, 0.001, 5000);
}


void TScene::meshSmooth()
{
	TAlgorithms::tSmooth(m_mesh, 1);
}

void TScene::setMeshView(bool face)
{
	m_faceView = face;
}

bool TScene::open(const QString& filename)
{
	/*m_mesh.clear();

	wchar_t wstr[256];
	int length = filename.toWCharArray(wstr);
	wstr[length] = 0;
	
	std::ifstream is;
	is.open(wstr);
	is >> m_mesh;
	return true;*/

	OpenMesh::IO::Options opt;
	if(!OpenMesh::IO::read_mesh(m_mesh, filename.toStdString(), opt))
		return false;
	if(!opt.check(OpenMesh::IO::Options::VertexNormal)){
		m_mesh.request_face_normals();
		m_mesh.update_normals();
		m_mesh.release_face_normals();
	}
	return true;
}

bool TScene::save(const QString& filename)
{
	/*wchar_t wstr[256];
	int length = filename.toWCharArray(wstr);
	wstr[length] = 0;
	
	std::ofstream os;
	os.open(wstr);
	os << m_mesh;
	return true;*/


	if(!OpenMesh::IO::write_mesh(m_mesh, filename.toStdString()))
		return false;
	return true;
}

void TScene::paint()
{
	glLoadIdentity();
	glMultMatrixf((m_mat * m_cam).constData());

	for(TriMesh::FaceIter fit = m_mesh.faces_begin(); 
		fit != m_mesh.faces_end();++ fit)
	{  
		glBegin(m_faceView ? GL_TRIANGLE_FAN : GL_LINE_LOOP);
		for(TriMesh::FaceVertexIter fvit = m_mesh.fv_begin(fit);
			fvit != m_mesh.fv_end(fit); ++fvit)
		{
			TriMesh::Normal n = m_mesh.normal(fvit.handle());
			glNormal3d(n[0], n[1], n[2]);
			TriMesh::Point p = m_mesh.point(fvit.handle());
			glVertex3d(p[0], p[1], p[2]);
		}
		glEnd();
	}
}

void TScene::setupViewport( int w, int h )
{
	m_canvasHeight = h;
	m_canvasWidth = w;

	int side = qMax(w, h);
	glViewport((w - side) / 2, (h - side) / 2, side, side);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();	
	glMultMatrixf(m_proj.constData());
	glMatrixMode(GL_MODELVIEW);
}

void TScene::camMoveView(const QVector3D& t)
{
	// like Google earth
	QVector3D tt = t * (m_cam_eye - m_cam_center).length() / 4.0;

	QVector3D xv = QVector3D::crossProduct((m_cam_center - m_cam_eye), m_cam_up).normalized();
	QVector3D yv = QVector3D::crossProduct(xv, (m_cam_center - m_cam_eye)).normalized();
	QVector3D xyTrans = xv * tt.x() + yv * tt.y();
	double r = ((m_cam_eye - m_cam_center).length() - tt.z()) / 
		(m_cam_eye + xyTrans - m_cam_center).length();
	m_cam_eye = (m_cam_eye + xyTrans - m_cam_center) * r + m_cam_center;
	m_cam_up = yv.normalized();

	m_cam.setToIdentity();
	m_cam.lookAt(m_cam_eye, m_cam_center, m_cam_up);
}

void TScene::camMoveCenter(const QVector3D& t)
{
	QVector3D tt = t * (m_cam_eye - m_cam_center).length() / 8.0;

	QVector3D xv = QVector3D::crossProduct((m_cam_center - m_cam_eye), m_cam_up).normalized();
	QVector3D yv = QVector3D::crossProduct(xv, (m_cam_center - m_cam_eye)).normalized();
	QVector3D zv = (m_cam_center  - m_cam_eye).normalized();
	QVector3D trans = xv * tt.x() + yv * tt.y() + zv * tt.z();
	m_cam_eye += trans;
	m_cam_center += trans;

	m_cam.setToIdentity();
	m_cam.lookAt(m_cam_eye, m_cam_center, m_cam_up);
}

void TScene::mapToZPlane()
{
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLdouble winX, winY, winZ;
	GLdouble posX, posY, posZ;

	int side = qMax(m_canvasWidth, m_canvasHeight);
	viewport[0] = (m_canvasWidth - side) / 2;
	viewport[1] = (m_canvasHeight - side) / 2;
	viewport[2] = viewport[3] = side;

	double eye_center_dist = (m_cam_eye - m_cam_center).length();
	QVector3D screenCenter(m_canvasWidth / 2.0, m_canvasHeight / 2.0, 0);

	double ratio = - 1.0;

	for(TriMesh::VertexIter vi  = m_mesh.vertices_begin();
		vi != m_mesh.vertices_end();
		++ vi)
	{
		TriMesh::Point pos = m_mesh.point(vi.handle());
		QVector3D qpos(pos[0], m_canvasHeight - pos[1], pos[2]);
		double m = (qpos - screenCenter).length();

		winX = qpos.x();
		winY = qpos.y();

        GLdouble mv[16], proj[16];
        auto mvMat = m_mat * m_cam;
        for (int i = 0; i < 16; i++) {
            mv[i] = mvMat.constData()[i];
            proj[i] = m_proj.constData()[i];
        }
		if(GL_TRUE == gluUnProject( winX, winY, 1, mv, 
			proj, 
			viewport, &posX, &posY, &posZ))
		{
			QVector3D p(posX, posY, posZ);			
			QVector3D r = m_cam_eye + (p - m_cam_eye) / 
				QVector3D::dotProduct(p - m_cam_eye, (m_cam_center - m_cam_eye).normalized()) * 
				(m_cam_center - m_cam_eye).length();
			if(ratio < 0)
				ratio = (r - m_cam_center).length() / m;
			r = r + (m_cam_center - m_cam_eye).normalized() * ratio * qpos.z();
			m_mesh.set_point(vi.handle(), TriMesh::Point(r.x(), r.y(), r.z()));
		}
	}
}

bool TScene::build( const QPolygonF& xyseeds, double stepLength)
{
	m_mesh.clear();

	if(xyseeds.size() <= 2)
		return false;

	/*QPolygonF polygon;
	polygon << xyseeds.first();
	for(int i = 1; i < xyseeds.size(); i++)
		if(QVector2D(polygon.last() - xyseeds[i]).length() >= stepLength)
			polygon << xyseeds[i];

	if(polygon.size() <= 2)
		return false;
	
	std::vector<TriMesh::VertexHandle> vhs;
	for(int i = 0; i < polygon.size(); i++)
		vhs.push_back(m_mesh.add_vertex(TriMesh::Point(polygon[i].x(), polygon[i].y(), 0)));*/

	TAlgorithms::tPartition(m_mesh, 
		TAlgorithms::tEqualize(m_mesh, xyseeds, stepLength));
	TAlgorithms::tDelaunay(m_mesh);
	TAlgorithms::tReTopo(m_mesh);
	TAlgorithms::tSew(m_mesh, 10);

	mapToZPlane();

	m_mesh.update_normals();
	
	return false;
}

