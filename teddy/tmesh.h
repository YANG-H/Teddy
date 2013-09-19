#ifndef TMESH_H
#define TMESH_H

#include <QMatrix4x4>

// need to define _USE_MATH_DEFINES and NOMINMAX in preprocess tab
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

class TMesh : public QObject
{
	typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;

public:
	TMesh(QObject *parent = 0);

	inline QMatrix4x4 matrix() const {return m_mat;}
	inline const qreal* matrixData() const {return m_mat.constData();}
	inline void rotate(double angle, const QVector3D& v)
	{QMatrix4x4 rm; rm.rotate(angle, v); m_mat = rm * m_mat;}
	inline void translate(const QVector3D& t) 
	{QMatrix4x4 tm; tm.translate(t); m_mat = tm * m_mat;}
	
	bool open(const QString& filename);
	bool save(const QString& filename);
	
	void paint();

private:
	TriMesh m_mesh;
	QMatrix4x4 m_mat;
};

#endif // TMESH_H
