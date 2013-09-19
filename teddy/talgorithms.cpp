#include "talgorithms.h"

#include <QVector3D>
#include <QVector2D>
#include <QQueue>
#include <QMap>
#include <QList>
#include <QtDebug>
#include <qmath.h>

#include <OpenMesh/Tools/Utils/StripifierT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/LoopT.hh>
#include <OpenMesh/Tools/Smoother/SmootherT.hh>
#include <OpenMesh/Tools/Smoother/LaplaceSmootherT.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>

#define qRoundAt(index, c) \
	(index < 0 ? c[-((-index) % c.size()) + c.size()] : c[(index + c.size()) % c.size()])
#define qRoundNear(a, b, size) \
	(abs(a - b) <= 1 || a == 0 && b == (size)-1 || a == (size)-1 && b== 0)

#define FT_JOINT	0
#define FT_SLEEVE	1
#define FT_TERMINAL 2
#define FT_ISOLATED 3
#define FT_NEW		5

namespace TAlgorithms
{
	// utilities
	inline double tDet(double* data)
	{
		double tmp1 = data[0*3+0] * (data[1*3+1]*data[2*3+2] - data[1*3+2]*data[2*3+1]);
		double tmp2 = data[0*3+1] * (data[1*3+0]*data[2*3+2] - data[1*3+2]*data[2*3+0]);
		double tmp3 = data[0*3+2] * (data[1*3+0]*data[2*3+1] - data[1*3+1]*data[2*3+0]);
		return tmp1 - tmp2 + tmp3;
	}

	inline bool tLeft(const TriMesh::Point& p,
		const TriMesh::Point& a, const TriMesh::Point& b)
	{
		double data[9] = {a[0], a[1], 1, b[0], b[1], 1, p[0], p[1], 1};
		return tDet(data) > 0;
	}

	inline bool tInTriangle(const TriMesh::Point& p,
		const TriMesh::Point& a, const TriMesh::Point& b, const TriMesh::Point& c)
	{
		bool lab = tLeft(p, a, b);
		bool lbc = tLeft(p, b, c);
		bool lca = tLeft(p, c, a);
		return lab == lbc && lbc == lca;
	}

	inline double tSqDist(const TriMesh::Point& p1, const TriMesh::Point& p2)
	{
		TriMesh::Point sub = p1 - p2;
		return sub[0] * sub[0] + sub[1] * sub[1] + sub[2] * sub[2];
	}

	inline bool tInCircle(const TriMesh::Point& p,
		const TriMesh::Point& a, const TriMesh::Point& b, const TriMesh::Point& c)
	{
		double data1[9] = { b[0], b[1], b[0]*b[0] + b[1]*b[1], 
			c[0], c[1], c[0]*c[0] + c[1]*c[1],
			p[0], p[1], p[0]*p[0] + p[1]*p[1]};
		double data2[9] = { a[0], a[1], a[0]*a[0] + a[1]*a[1], 
			c[0], c[1], c[0]*c[0] + c[1]*c[1],
			p[0], p[1], p[0]*p[0] + p[1]*p[1]};
		double data3[9] = { a[0], a[1], a[0]*a[0] + a[1]*a[1],
			b[0], b[1], b[0]*b[0] + b[1]*b[1], 					    
			p[0], p[1], p[0]*p[0] + p[1]*p[1]};
		double data4[9] = { a[0], a[1], a[0]*a[0] + a[1]*a[1],
			b[0], b[1], b[0]*b[0] + b[1]*b[1], 					    
			c[0], c[1], c[0]*c[0] + c[1]*c[1]};
		double result = - tDet(data1) + tDet(data2) - tDet(data3) + tDet(data4);
		return result > 0;

	/*	TriMesh::Point center = (a+c)/2.0;
		double diamSq = tSqDist(a, c);
		return 4 * tSqDist(p, center) < diamSq && 4 * tSqDist(b, center) < diamSq;*/
	}

	inline TriMesh::Point tPoint(const QPointF& p)
	{
		return TriMesh::Point(p.x(), p.y(), 0);
	}

	inline TriMesh::Point tPoint(const QVector3D& p)
	{
		return TriMesh::Point(p.x(), p.y(), p.z());
	}

	inline void tAdjustAllVertices(TriMesh& mesh)
	{
		for(TriMesh::VertexIter vi = mesh.vertices_begin();
			vi != mesh.vertices_end();
			++ vi)
			mesh.adjust_outgoing_halfedge(vi.handle());
	}

	template <typename T>
	inline void tFillFacesProperty(TriMesh& mesh, OpenMesh::FPropHandleT<T> prop, T v)
	{
		for(TriMesh::FaceIter fi = mesh.faces_begin();
			fi != mesh.faces_end();
			++ fi)
			mesh.property(prop, fi.handle()) = v;
	}
	template <typename T>
	inline void tFillVerticesProperty(TriMesh& mesh, OpenMesh::VPropHandleT<T> prop, T v)
	{
		for(TriMesh::VertexIter vi = mesh.vertices_begin();
			vi != mesh.vertices_end();
			++ vi)
			mesh.property(prop, vi.handle()) = v;
	}
	template <typename T>
	inline void tFillEdgesProperty(TriMesh& mesh, OpenMesh::EPropHandleT<T> prop, T v)
	{
		for(TriMesh::EdgeIter ei = mesh.edges_begin();
			ei != mesh.edges_end();
			++ ei)
			mesh.property(prop, ei.handle()) = v;
	}
	template <typename T>
	inline void tFillHalfedgesProperty(TriMesh& mesh, OpenMesh::HPropHandleT<T> prop, T v)
	{
		for(TriMesh::HalfedgeIter hi = mesh.halfedges_begin();
			hi != mesh.halfedges_end();
			++ hi)
			mesh.property(prop, hi.handle()) = v;
	}


	//////////////////////////////////////////////////////////////////////////
	// polygon partition
	//////////////////////////////////////////////////////////////////////////
	void tPartition( TriMesh& mesh, const QList<TriMesh::VertexHandle>& vhs )
	{
		QQueue<QList<int> > vhIndexGroupQ;
		QList<int> indexG;
		for(int i = 0; i < vhs.size(); i++)
			indexG.push_back(i);
		vhIndexGroupQ.push_back(indexG);

		while(!vhIndexGroupQ.empty())
		{
			QList<int> is = vhIndexGroupQ.first();
			vhIndexGroupQ.pop_front();

			Q_ASSERT(is.size() >= 3);
			if(is.size() <= 2)
				continue;

			if(is.size() == 3)
				mesh.add_face(vhs[is[0]], vhs[is[1]], vhs[is[2]]);
			else{
				// leftmost
				int leftmostII = 0;
				TriMesh::Point leftmostP = mesh.point(vhs[is[leftmostII]]);
				for(int i = 0; i < is.size(); i++){
					TriMesh::Point p = mesh.point(vhs[is[i]]);
					if(p[0] < leftmostP[0]){
						leftmostII = i;
						leftmostP = p;
					}
				}

				int leftmostPrevII = (leftmostII + is.size() - 1) % is.size();
				int leftmostNextII = (leftmostII + 1) % is.size();
				TriMesh::Point a = mesh.point(vhs[is[leftmostPrevII]]);
				TriMesh::Point b = mesh.point(vhs[is[leftmostNextII]]);

				int innerLeftmostII = -1;
				TriMesh::Point innerLeftmostP;
				for(int i = 0; i < is.size(); i++){
					if(qRoundNear(i, leftmostII, is.size()))
						continue;
					TriMesh::Point p = mesh.point(vhs[is[i]]);
					if(tInTriangle(p, a, leftmostP, b))
					{
						if(innerLeftmostII == -1){
							innerLeftmostII = i;
							innerLeftmostP = p;
						}else if(p[0] < innerLeftmostP[0]){
							innerLeftmostII = i;
							innerLeftmostP = p;
						}
					}
				}

				int split1 = leftmostII;
				int split2 = innerLeftmostII;
				if(innerLeftmostII < 0){
					split1 = leftmostPrevII;
					split2 = leftmostNextII;
				}

				Q_ASSERT(split1 != split2);

				QList<int> part1, part2;

				for(int i = split1; i != split2; i = (i + 1) % is.size())
					part1.push_back(is[i]);
				part1.push_back(is[split2]);
				for(int i = split2; i != split1; i = (i + 1) % is.size())
					part2.push_back(is[i]);
				part2.push_back(is[split1]);

				Q_ASSERT(part1.size() >= 3);
				Q_ASSERT(part2.size() >= 3);

				is.clear();

				vhIndexGroupQ.push_back(part1);
				vhIndexGroupQ.push_back(part2);
			}
		}
	}

	void tLegalize(TriMesh& mesh, const TriMesh::VertexHandle& vh, 
		const TriMesh::HalfedgeHandle& testh)
	{
		TriMesh::HalfedgeHandle oppoh = mesh.opposite_halfedge_handle(testh);

		TriMesh::VertexHandle sidev1  = mesh.to_vertex_handle(testh);
		TriMesh::VertexHandle sidev2 = mesh.to_vertex_handle(oppoh);
		TriMesh::VertexHandle oppov = mesh.to_vertex_handle(mesh.next_halfedge_handle(oppoh));

		TriMesh::Point p = mesh.point(vh);
		TriMesh::Point sidep1 = mesh.point(sidev1);
		TriMesh::Point sidep2 = mesh.point(sidev2);
		TriMesh::Point oppop = mesh.point(oppov);

		if(tInCircle(p, sidep1, sidep2, oppop)){
			TriMesh::EdgeHandle eh = mesh.edge_handle(testh);
			if(mesh.is_flip_ok(eh)){
				TriMesh::HalfedgeHandle h1, h2;
				h1 = mesh.next_halfedge_handle(oppoh);
				h2 = mesh.next_halfedge_handle(h1);
				mesh.flip(eh);

				// more legalize
				tLegalize(mesh, vh, h1);
				tLegalize(mesh, vh, h2);
			}
		} 
	}

	//////////////////////////////////////////////////////////////////////////
	// constrained delaunay triangulation
	//////////////////////////////////////////////////////////////////////////
	void tDelaunay(TriMesh& mesh)
	{
		for(int i = 0; i < 4; i++)
		{
		for(TriMesh::VertexIter vi = mesh.vertices_begin();
			vi != mesh.vertices_end();
			++ vi)
		{
			int i = 0;
			for(TriMesh::VertexOHalfedgeIter oh = mesh.voh_begin(vi.handle());
				oh != mesh.voh_end(vi.handle());
				++ oh)
			{
				TriMesh::HalfedgeHandle testh = mesh.next_halfedge_handle(oh.handle());
				tLegalize(mesh, vi.handle(), testh);
				i++;
			}
			mesh.adjust_outgoing_halfedge(vi.handle());
			//qDebug() << i;
		}
		}

		mesh.delete_isolated_vertices();
		mesh.garbage_collection();
	}

	// classify faces as Joint/Sleeve/Terminal face
	void tClassifyFaces(TriMesh& mesh, OpenMesh::FPropHandleT<int> faceType)
	{
		tFillFacesProperty(mesh, faceType, 0);
		for(TriMesh::EdgeIter ei = mesh.edges_begin();
			ei != mesh.edges_end();
			++ ei)
		{
			TriMesh::EdgeHandle eh = ei.handle();
			TriMesh::FaceHandle fh1 = mesh.face_handle(mesh.halfedge_handle(eh, 0));
			TriMesh::FaceHandle fh2 = mesh.face_handle(mesh.halfedge_handle(eh, 1));

			if(!mesh.is_valid_handle(fh1))
				mesh.property(faceType, fh2) = mesh.property(faceType, fh2) + 1;
			if(!mesh.is_valid_handle(fh2))
				mesh.property(faceType, fh1) = mesh.property(faceType, fh1) + 1;
		}
	}

	// split joint face into tree triangles
	QList<TriMesh::FaceHandle> 
		tSplitJointFaces(TriMesh& mesh, OpenMesh::FPropHandleT<int> faceType)
	{
		QList<TriMesh::FaceHandle> terminalFaces;
		for(TriMesh::FaceIter fi = mesh.faces_begin();
			fi != mesh.faces_end();
			++ fi)
		{
			TriMesh::FaceHandle fh = fi.handle();
			int t = mesh.property(faceType, fh);
			if(t == FT_TERMINAL){
				//mesh.set_color(fh, FT_TERMINAL_COLOR);
				terminalFaces.push_back(fh);
			}else if(t == FT_SLEEVE){
				//mesh.set_color(fh, FT_SLEEVE_COLOR);
			}else if(t == FT_JOINT){
				//mesh.set_color(fh, FT_JOINT_COLOR);
				// split joint faces
				// calc center
				TriMesh::Point c(0, 0, 0);
				for(TriMesh::FaceVertexIter fvi = mesh.fv_begin(fh);
					fvi != mesh.fv_end(fh);
					++ fvi)
					c += mesh.point(fvi.handle());
				c /= 3.0;

				TriMesh::VertexHandle nvh = mesh.add_vertex(c);
				mesh.split(fh, nvh);
				for(TriMesh::VertexFaceIter vfi = mesh.vf_begin(nvh);
					vfi != mesh.vf_end(nvh);
					++ vfi)
					mesh.property(faceType, vfi.handle()) = FT_NEW;
			}
		}

		return terminalFaces;
	}

	inline bool tIsInternalVertex(TriMesh& mesh, TriMesh::VertexHandle vh){
		for(TriMesh::VertexEdgeIter vei = mesh.ve_begin(vh);
			vei != mesh.ve_end(vh);
			++ vei)
			if(mesh.is_boundary(vei.handle()))
				return false;
		return true;
	}

	// classify vertices as internal/edge vertex
	void tClassifyVertices(TriMesh& mesh, OpenMesh::VPropHandleT<bool> vIsInternal){
		for(TriMesh::VertexIter vi = mesh.vertices_begin();
			vi != mesh.vertices_end();
			++ vi){
				mesh.property(vIsInternal, vi.handle()) = 
					tIsInternalVertex(mesh, vi.handle());
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// build fans
	//////////////////////////////////////////////////////////////////////////
	void tReTopo(TriMesh& mesh)
	{
		QList<TriMesh::FaceHandle> terminalFaces;

		OpenMesh::FPropHandleT<int> faceType;
		OpenMesh::FPropHandleT<bool> faceChecked;
		mesh.add_property(faceType, "face-type");
		mesh.add_property(faceChecked, "face-checked");

		tFillFacesProperty(mesh, faceChecked, false);

		// classify faces
		tClassifyFaces(mesh, faceType);
		terminalFaces = tSplitJointFaces(mesh, faceType);
		tClassifyFaces(mesh, faceType);
		tAdjustAllVertices(mesh);

		// split terminal faces and sleeve faces
		for(int i = 0; i < terminalFaces.size(); i++)
		{
			if(mesh.property(faceType, terminalFaces[i]) != FT_TERMINAL)
				continue;
			if(mesh.property(faceChecked, terminalFaces[i]))
				continue;

			QList<TriMesh::VertexHandle> collectedVs; // vertices recorded in clockwise
			QList<TriMesh::FaceHandle> collectedFs; // faces recorded, will be deleted		

			// insert the boundary point
			for(TriMesh::FaceVertexIter fvi = mesh.fv_begin(terminalFaces[i]);
				fvi != mesh.fv_end(terminalFaces[i]);
				++ fvi)
			{
				if(mesh.valence(fvi.handle()) == 2){
					collectedVs.push_back(fvi.handle());
					break;
				}
			}
			Q_ASSERT(collectedVs.size() == 1);		

			forever {
				bool mustStop = false;

				// get curfh
				if(collectedFs.empty())
					collectedFs.push_back(terminalFaces[i]);
				else{
					TriMesh::FaceHandle nextFh;
					for(TriMesh::FaceFaceIter ffi = mesh.ff_begin(collectedFs.last());
						ffi != mesh.ff_end(collectedFs.last());
						++ ffi)
					{
						if(!collectedFs.contains(ffi.handle())
							&& mesh.is_valid_handle(ffi.handle())
							&& mesh.property(faceType, ffi.handle()) != FT_NEW )
						{
							nextFh = ffi.handle();
							break;
						}
					}
					if(mesh.is_valid_handle(nextFh))
						collectedFs.push_back(nextFh);
					else
						mustStop = true;
				}
				TriMesh::FaceHandle curfh = collectedFs.last();
				mesh.property(faceChecked, curfh) = true;

				int t = mesh.property(faceType, curfh);			

				if(t == FT_SLEEVE || t == FT_TERMINAL){ // sleeve face or terminal face
					// get next hh2check
					// this edge should not be on boundary
					TriMesh::HalfedgeHandle hh2check;
					for(TriMesh::FaceHalfedgeIter fhi = mesh.fh_begin(curfh);
						fhi != mesh.fh_end(curfh);
						++ fhi)
					{
						TriMesh::FaceHandle oppof = mesh.opposite_face_handle(fhi.handle());
						if(mesh.is_valid_handle(oppof)
							&& !collectedFs.contains(oppof))
						{
							hh2check = fhi.handle();
							break;
						}
					}
					Q_ASSERT(mesh.is_valid_handle(hh2check) && !mesh.is_boundary(hh2check));

					// add new vertices
					TriMesh::VertexHandle right, left; 
					right = mesh.from_vertex_handle(hh2check); 
					left = mesh.to_vertex_handle(hh2check);
					Q_ASSERT(mesh.is_valid_handle(left) && mesh.is_valid_handle(right));
					if(!collectedVs.contains(left) && mesh.is_valid_handle(left))
						collectedVs.push_front(left);
					if(!collectedVs.contains(right) && mesh.is_valid_handle(right))
						collectedVs.push_back(right);

					// circle check
					bool inCircle = true;
					TriMesh::Point center = (mesh.point(right) + mesh.point(left)) / 2.0;
					double diamSq = tSqDist(mesh.point(right), mesh.point(left));
					for(int i = 1; i < collectedVs.size()-1; i++)
					{
						TriMesh::VertexHandle vh = collectedVs[i];
						if(tSqDist(mesh.point(vh), center) * 4 > diamSq){
							inCircle = false;
							break;
						}
					}
					if(inCircle && !mustStop){	
						continue;
					}else{
						// delete all collected faces
						foreach(TriMesh::FaceHandle fh, collectedFs)
							mesh.delete_face(fh, false); // don't delete vertices!!!
						collectedFs.clear();
						for(int i = 0; i < collectedVs.size(); i++)
							mesh.adjust_outgoing_halfedge(collectedVs[i]);

						// add a new vertex on the middle point of hh2check
						TriMesh::VertexHandle finalvh = mesh.add_vertex(center);
						mesh.split(mesh.edge_handle(hh2check), finalvh);

						// rebuild fans
						for(int i = 1; i < collectedVs.size(); i++)
							mesh.add_face(finalvh, collectedVs[i-1], collectedVs[i]);

						mesh.adjust_outgoing_halfedge(finalvh);
						for(int i = 0; i < collectedVs.size(); i++)
							mesh.adjust_outgoing_halfedge(collectedVs[i]);

						// update type
						for(TriMesh::VertexFaceIter vfi = mesh.vf_begin(finalvh);
							vfi != mesh.vf_end(finalvh);
							++ vfi)
						{
							mesh.property(faceChecked, vfi.handle()) = true;
							mesh.property(faceType, vfi.handle()) = FT_NEW;
						}
						break;
					}
				}	

				if(t == FT_JOINT){ // joint face
					// if meets split joint face, remesh and break immediately
					// find the final vertex
					TriMesh::VertexHandle finalvh;
					for(TriMesh::FaceVertexIter fvi = mesh.fv_begin(curfh);
						fvi != mesh.fv_end(curfh);
						++ fvi)
					{
						if(!mesh.is_boundary(fvi.handle())){
							finalvh = fvi.handle();
							break;
						}
					}
					Q_ASSERT(mesh.is_valid_handle(finalvh));

					// now remesh
					foreach(TriMesh::FaceHandle fh, collectedFs)
						mesh.delete_face(fh, false);
					collectedFs.clear();
					mesh.adjust_outgoing_halfedge(finalvh);
					for(int i = 0; i < collectedVs.size(); i++)
						mesh.adjust_outgoing_halfedge(collectedVs[i]);

					for(int i = 1; i < collectedVs.size(); i++)
						mesh.add_face(finalvh, collectedVs[i-1], collectedVs[i]);

					mesh.adjust_outgoing_halfedge(finalvh);
					for(int i = 0; i < collectedVs.size(); i++)
						mesh.adjust_outgoing_halfedge(collectedVs[i]);

					// update type
					for(TriMesh::VertexFaceIter vfi = mesh.vf_begin(finalvh);
						vfi != mesh.vf_end(finalvh);
						++ vfi)
					{
						mesh.property(faceChecked, vfi.handle()) = true;
						mesh.property(faceType, vfi.handle()) = FT_NEW;
					}
					break;
				}
			}		
		}

		mesh.remove_property(faceType);
		mesh.remove_property(faceChecked);
		mesh.garbage_collection();
	}

	//////////////////////////////////////////////////////////////////////////
	// sew strips, build chord axis and construct mesh body
	//////////////////////////////////////////////////////////////////////////
	void tSew(TriMesh& mesh, int sep)
	{
		static const TriMesh::Point z(0, 0, 1);

		OpenMesh::VPropHandleT<bool> vIsInternal;
		OpenMesh::VPropHandleT<double> vHeight;
		mesh.add_property(vIsInternal, "vertex-type");
		mesh.add_property(vHeight, "vertex-height");

		tClassifyVertices(mesh, vIsInternal);

		// split all cross edges
		for(TriMesh::HalfedgeIter hi = mesh.halfedges_begin();
			hi != mesh.halfedges_end();
			++ hi)
		{
			TriMesh::HalfedgeHandle hh = hi.handle();
			TriMesh::VertexHandle v1 = mesh.from_vertex_handle(hh);
			TriMesh::VertexHandle v2 = mesh.to_vertex_handle(hh);

			if(mesh.is_valid_handle(mesh.opposite_face_handle(hh)) 
				&& mesh.is_valid_handle(mesh.face_handle(hh))
				&& !mesh.property(vIsInternal, v1) && !mesh.property(vIsInternal, v2))
			{ // split cross edge
				TriMesh::Point p1 = mesh.point(v1);
				TriMesh::Point p2 = mesh.point(v2);
				TriMesh::Point center = (p1 + p2) / 2.0;
				TriMesh::VertexHandle nvh = mesh.add_vertex(center);
				mesh.split(mesh.edge_handle(hh), nvh);
				mesh.property(vIsInternal, nvh) = true;
			}
		} 

		// calc new internal vertices new pos
		for(TriMesh::VertexIter vi = mesh.vertices_begin();
			vi != mesh.vertices_end();
			++ vi)
		{
			if(mesh.property(vIsInternal, vi.handle())){
				TriMesh::Point newPos(0, 0, 0);
				int n = 0;
				for(TriMesh::VertexVertexIter vvi = mesh.vv_begin(vi.handle());
					vvi != mesh.vv_end(vi.handle());
					++ vvi)
				{
					if(!mesh.property(vIsInternal, vvi.handle())){
						newPos += mesh.point(vvi.handle());
						n ++;
					}
				}
				newPos /= n;
				mesh.set_point(vi.handle(), newPos);
			}
		}

		// calc internal vertices heights
		for(TriMesh::VertexIter vi = mesh.vertices_begin();
			vi != mesh.vertices_end();
			++ vi)
		{
			double height = 0;
			if(mesh.property(vIsInternal, vi.handle())){
				TriMesh::Point pos = mesh.point(vi.handle());
				int externalNeighborNum = 0;
				for(TriMesh::VertexVertexIter vvi = mesh.vv_begin(vi.handle());
					vvi != mesh.vv_end(vi.handle());
					++ vvi)
				{
					if(!mesh.property(vIsInternal, vvi.handle())){
						height += qSqrt(tSqDist(mesh.point(vvi.handle()), pos));
						externalNeighborNum ++;
					}
				}
				Q_ASSERT(externalNeighborNum != 0);
				height /= externalNeighborNum;
				pos += z * height;
				mesh.set_point(vi.handle(), pos);
			}
			mesh.property(vHeight, vi.handle()) = height;
		}

		// collect old faces and vertices
		QList<TriMesh::FaceHandle> oldFaces;
		for(TriMesh::FaceIter fi = mesh.faces_begin();
			fi != mesh.faces_end();
			++ fi)
			oldFaces.push_back(fi.handle());
		QList<TriMesh::VertexHandle> oldVertices;
		for(TriMesh::VertexIter vi = mesh.vertices_begin();
			vi != mesh.vertices_end();
			++ vi)
			oldVertices.push_back(vi.handle());

		// duplicate all internal vertices
		QMap<TriMesh::VertexHandle, TriMesh::VertexHandle> intvmap;
		foreach(TriMesh::VertexHandle vh, oldVertices){
			bool isInternal = mesh.property(vIsInternal, vh);
			if(isInternal){
				TriMesh::Point pos = mesh.point(vh);
				TriMesh::VertexHandle mirror =
					mesh.add_vertex(pos - z * 2 * mesh.property(vHeight, vh));
				mesh.property(vHeight, mirror) = - mesh.property(vHeight, vh);
				mesh.property(vIsInternal, mirror) = true;
				intvmap.insert(vh, mirror);
			}
		}

		// construct e-> vs mapping
		QMap<TriMesh::EdgeHandle, QList<TriMesh::VertexHandle> > evmap1, evmap2;
		for(TriMesh::EdgeIter ei = mesh.edges_begin();
			ei != mesh.edges_end();
			++ ei)
		{
			TriMesh::EdgeHandle eh = ei.handle();
			// judge if is half cross edge
			TriMesh::VertexHandle v1 = mesh.from_vertex_handle(mesh.halfedge_handle(eh, 0));
			TriMesh::VertexHandle v2 = mesh.from_vertex_handle(mesh.halfedge_handle(eh, 1));
			if(mesh.property(vIsInternal, v1) == mesh.property(vIsInternal, v2)) // isn't
				continue;

			// update height
			TriMesh::Point p1 = mesh.point(v1);
			TriMesh::Point p2 = mesh.point(v2);

			if(mesh.property(vIsInternal, v2)) // v2 is internal
				qSwap(p1, p2); // keep p1 internal

			TriMesh::Point v = p1 - p2;
			qreal stepAngle = 3.141592658 / 2.0 / double(sep);
			QList<TriMesh::VertexHandle> vs, vsm;
			for(int i = 1; i < sep; i++){
				qreal cosR = qCos(stepAngle * i);
				qreal sinR = qSin(stepAngle * i);

				TriMesh::Point curPos = p2 + 
					TriMesh::Point(v[0] * (1-sinR), v[1] * (1-sinR), v[2] * (cosR)) ;

				TriMesh::VertexHandle vh = mesh.add_vertex(curPos);
				mesh.property(vIsInternal, vh) = true;

				TriMesh::VertexHandle vhmirror = 
					mesh.add_vertex(TriMesh::Point(curPos[0], curPos[1], - curPos[2]));
				mesh.property(vIsInternal, vhmirror) = true;

				vs.append(vh);
				vsm.append(vhmirror);
			}
			evmap1.insert(eh, vs);
			evmap2.insert(eh, vsm);
		}

		// collect new faces
		QList<TriMesh::VertexHandle> vf1, vf2, vf3;
		foreach(TriMesh::FaceHandle oldf, oldFaces)
		{
			TriMesh::FaceVertexIter fvi = mesh.fv_begin(oldf);
			TriMesh::VertexHandle v[3];
			v[0] = fvi.handle();
			v[1] = (++ fvi).handle();
			v[2] = (++ fvi).handle();

			int isInternal[3];
			isInternal[0] = mesh.property(vIsInternal, v[0]) ? 1 : 0;
			isInternal[1] = mesh.property(vIsInternal, v[1]) ? 1 : 0;
			isInternal[2] = mesh.property(vIsInternal, v[2]) ? 1 : 0;

			int sum = isInternal[0] + isInternal[1] + isInternal[2];
			Q_ASSERT(sum == 1 || sum == 2);

			bool isEdgeFace = (isInternal[0] + isInternal[1] + isInternal[2] == 1);

			TriMesh::VertexHandle pointedV;
			if(isEdgeFace)
			{
				pointedV = isInternal[0] ? v[0] : (isInternal[1] ? v[1] : v[2]);
				TriMesh::VertexHandle pointedVm = intvmap.value(pointedV);

				TriMesh::HalfedgeHandle h1, h2;
				for(TriMesh::FaceHalfedgeIter fhi = mesh.fh_begin(oldf);
					fhi != mesh.fh_end(oldf);
					++ fhi)
				{
					if(mesh.to_vertex_handle(fhi.handle()) == pointedV)
						h2 = fhi.handle();
					else if(mesh.from_vertex_handle(fhi.handle()) == pointedV)
						h1 = fhi.handle();
				}

				QList<TriMesh::VertexHandle> newv1 = evmap1.value(mesh.edge_handle(h1));
				newv1.push_back(mesh.to_vertex_handle(h1));
				QList<TriMesh::VertexHandle> newv2 = evmap1.value(mesh.edge_handle(h2));
				newv2.push_back(mesh.from_vertex_handle(h2));

				QList<TriMesh::VertexHandle> newv1m = evmap2.value(mesh.edge_handle(h1));
				newv1m.push_back(mesh.to_vertex_handle(h1));
				QList<TriMesh::VertexHandle> newv2m = evmap2.value(mesh.edge_handle(h2));
				newv2m.push_back(mesh.from_vertex_handle(h2));

				// start making faces
				//mesh.add_face(newv1.first(), pointedV, newv2.first());
				//mesh.add_face(newv2m.first(), pointedVm, newv1m.first());
				vf1 << newv1.first() << newv2m.first(); 
				vf2 << pointedV		 << pointedVm;
				vf3 << newv2.first() << newv1m.first();

				for(int i = 1; i < sep; i++)
				{
					//mesh.add_face(newv1[i], newv1[i-1], newv2[i-1]);
					//mesh.add_face(newv1[i], newv2[i-1], newv2[i]);
					//mesh.add_face(newv1m[i], newv2m[i-1], newv1m[i-1]);
					//mesh.add_face(newv1m[i], newv2m[i], newv2m[i-1]);
					vf1 << newv1[i]   << newv1[i]   << newv1m[i]   << newv1m[i];
					vf2 << newv1[i-1] << newv2[i-1] << newv2m[i-1] << newv2m[i];
					vf3 << newv2[i-1] << newv2[i]   << newv1m[i-1] << newv2m[i-1];
				}

			}else
			{
				pointedV = (!isInternal[0]) ? v[0] : ((!isInternal[1]) ? v[1] : v[2]);

				TriMesh::HalfedgeHandle h1, h2;
				for(TriMesh::FaceHalfedgeIter fhi = mesh.fh_begin(oldf);
					fhi != mesh.fh_end(oldf);
					++ fhi)
				{
					TriMesh::EdgeHandle e = mesh.edge_handle(fhi.handle());
					if(mesh.to_vertex_handle(fhi.handle()) == pointedV)
						h1 = fhi.handle();
					else if(mesh.from_vertex_handle(fhi.handle()) == pointedV)
						h2 = fhi.handle();
				}
				//Q_ASSERT(mesh.is_valid_handle(h1) && mesh.is_valid_handle(h2));
				//Q_ASSERT(mesh.is_valid_handle(mesh.edge_handle(h1)));
				//Q_ASSERT(mesh.is_valid_handle(mesh.edge_handle(h2)));

				TriMesh::EdgeHandle e1 = mesh.edge_handle(h1);
				TriMesh::EdgeHandle e2 = mesh.edge_handle(h2);

				QList<TriMesh::VertexHandle> newv1 = evmap1.value(mesh.edge_handle(h1));
				newv1.push_front(mesh.from_vertex_handle(h1));
				QList<TriMesh::VertexHandle> newv2 = evmap1.value(mesh.edge_handle(h2));
				newv2.push_front(mesh.to_vertex_handle(h2));

				QList<TriMesh::VertexHandle> newv1m = evmap2.value(mesh.edge_handle(h1));
				newv1m.push_front(intvmap[mesh.from_vertex_handle(h1)]);
				QList<TriMesh::VertexHandle> newv2m = evmap2.value(mesh.edge_handle(h2));
				newv2m.push_front(intvmap[mesh.to_vertex_handle(h2)]);

				// start making faces
				for(int i = 1; i < sep; i++){
					//mesh.add_face(newv1[i], newv1[i-1], newv2[i-1]);
					//mesh.add_face(newv1[i], newv2[i-1], newv2[i]);
					//mesh.add_face(newv1m[i], newv2m[i-1], newv1m[i-1]);
					//mesh.add_face(newv1m[i], newv2m[i], newv2m[i-1]);
					vf1 << newv1[i] << newv1[i] << newv1m[i] << newv1m[i];
					vf2 << newv1[i-1] << newv2[i-1] << newv2m[i-1] << newv2m[i];
					vf3 << newv2[i-1] << newv2[i] << newv1m[i-1] << newv2m[i-1];
				}

				//mesh.add_face(newv2.last(), pointedV, newv1.last());
				//mesh.add_face(newv1m.last(), pointedV, newv2m.last());
				vf1 << newv2.last() << newv1m.last();
				vf2 << pointedV << pointedV;
				vf3 << newv1.last() << newv2m.last();
			}
		}

		foreach(TriMesh::FaceHandle oldf, oldFaces)
			mesh.delete_face(oldf, false);

		for(int i = 0; i < vf1.size(); i++)
			mesh.add_face(vf3[i], vf2[i], vf1[i]);

		mesh.delete_isolated_vertices();
		tAdjustAllVertices(mesh);

		// adjust new vertices positions
		for(int i = 0; i < 4; i++){
			for(TriMesh::VertexIter vi = mesh.vertices_begin();
				vi != mesh.vertices_end();
				++ vi)
			{
				//if(mesh.property(vType, vi.handle())){
					TriMesh::Point pos(0, 0, 0);// mesh.point(vi.handle());
					int neighbourNum = 0;
					for(TriMesh::VertexVertexIter vvi = mesh.vv_begin(vi.handle());
						vvi != mesh.vv_end(vi.handle());
						++ vvi){pos += mesh.point(vvi.handle()); neighbourNum ++;}
					pos /= double(neighbourNum);
					mesh.set_point(vi.handle(), pos);
				//}
			}
		}

		mesh.remove_property(vIsInternal);
		mesh.remove_property(vHeight);
		mesh.garbage_collection();
	}

	void tSmooth(TriMesh& mesh, int rep)
	{
		OpenMesh::Smoother::JacobiLaplaceSmootherT<TriMesh> smoother(mesh);
		smoother.smooth(rep);
	}

	QList<TriMesh::VertexHandle> tEqualize( TriMesh& mesh, 
		const QPolygonF& poly, double dist /*= 10.0*/ )
	{
		QList<TriMesh::VertexHandle> equalVs;
		
		if(poly.empty())
			return equalVs;
		
		equalVs << mesh.add_vertex(tPoint(poly.first()));
		if(poly.size() == 1)
			return equalVs;

		QPointF last = poly.first();
		double distRemained = 0;
		for(int i = 1; i <= poly.size(); i++){
			QVector2D newV = QVector2D(poly[i % poly.size()] - poly[i-1]);
			double newDist = newV.length();
			if(distRemained + newDist < dist){
				distRemained += newDist;
				continue;
			}
			// distremained + newdist >= dist
			double d = dist - distRemained;
			// 0 < d <= newdist
			for(; d <= newDist; d += dist){
				QPointF np = newV.normalized().toPointF() * d + poly[i-1];
				equalVs << mesh.add_vertex(tPoint(np));

				qDebug() << "##" << QVector2D(np - last).length();
				last = np;
				distRemained = newDist - d;
			}			
		}

		return equalVs;
	}
}
