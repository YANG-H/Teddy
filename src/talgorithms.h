#ifndef TALGORITHMS_H
#define TALGORITHMS_H

#include <QList>
#include <QPolygonF>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

struct TMeshTraits : public OpenMesh::DefaultTraits
{
	VertexAttributes( OpenMesh::Attributes::Normal |  
		OpenMesh::Attributes::Status);
	FaceAttributes( OpenMesh::Attributes::Normal | 
		OpenMesh::Attributes::Status | 
		OpenMesh::Attributes::Color);
	HalfedgeAttributes( OpenMesh::Attributes::Status);
	EdgeAttributes(OpenMesh::Attributes::Status);
};

typedef OpenMesh::TriMesh_ArrayKernelT<TMeshTraits> TriMesh;

namespace TAlgorithms
{
	QList<TriMesh::VertexHandle> tEqualize(TriMesh& mesh, 
		const QPolygonF& poly, double dist = 10.0);
	void tPartition(TriMesh& mesh, const QList<TriMesh::VertexHandle>& vhs);
	void tDelaunay(TriMesh& mesh);
	void tDelaunay2(TriMesh& mesh);
	void tReTopo(TriMesh& mesh);
	void tSew(TriMesh& mesh, int sep);
	void tSmooth(TriMesh& mesh, int rep);
	bool tIntersectWithRay(TriMesh& mesh, const TriMesh::Point& rayP, const TriMesh::Point& rayD);
}

#endif // TALGORITHMS_H
