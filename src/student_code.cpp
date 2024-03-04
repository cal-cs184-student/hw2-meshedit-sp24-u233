#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
      size_t numPoints = points.size();

      if (numPoints == 1) {
          return points;
      }
      vector<Vector2D> interpoints;

      for (size_t i = 0; i < numPoints - 1; i ++) {
          interpoints.push_back((1.0 - t) * points[i] + t * points[i + 1]);
      }
    return interpoints;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
      size_t numPoints = points.size();

      if (numPoints == 1) {
          return points;
      }
      vector<Vector3D> interpoints;

      for (size_t i = 0; i < numPoints - 1; i++) {
          interpoints.push_back((1.0 - t) * points[i] + t * points[i + 1]);
      }
      return interpoints;

  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
      vector<Vector3D> currentPoints = points;
      while (currentPoints.size() > 1) {
          currentPoints = evaluateStep(currentPoints, t);
      }
      
    return currentPoints[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {
      vector<Vector3D> moving;
      for (vector<Vector3D> row : controlPoints) {
          moving.push_back(evaluate1D(row, u));
      }

    return evaluate1D(moving,v);
  }

  Vector3D Vertex::normal( void ) const
  {
      Vector3D N(0., 0., 0.);
      HalfedgeCIter h = halfedge();

      do {
          Vector3D v1 = h->twin()->vertex()->position - this->position;
          Vector3D v2 = h->next()->twin()->vertex()->position - h->twin()->vertex()->position;

          N += cross(v1,v2);

          h = h->twin()->next(); 
      } while (h != halfedge());

      //Vector3D N_norm = N.normalize();


      return N.unit();





      /*
      Vector3D Face::normal(void) const
      {
          Vector3D N(0., 0., 0.);

          HalfedgeCIter h = halfedge();
          do
          {
              Vector3D pi = h->vertex()->position;
              Vector3D pj = h->next()->vertex()->position;

              N += cross(pi, pj);

              h = h->next();
          } while (h != halfedge());

          return N.unit();
      }*/


    //         HalfedgeCIter halfedge( void ) const { return _halfedge; }

    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    //return Vector3D();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
      if (e0->isBoundary()) {
          return EdgeIter();
      }
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin(); 
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next(); 
      HalfedgeIter h6 = h1->twin(); 
      HalfedgeIter h7 = h2->twin(); 
      HalfedgeIter h8 = h4->twin(); 
      HalfedgeIter h9 = h5->twin(); 

      

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex(); 
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex(); 

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      if (f0->isBoundary() || f1->isBoundary()) {
          return EdgeIter();
      }


      //Alter pointers: 

      h0->setNeighbors(h1, h3, v3, e0, f0); //yes
      h1->setNeighbors(h2, h7, v2, e2, f0); //yes
      h2->setNeighbors(h0, h8, v0, e3, f0); //y
      h3->setNeighbors(h4, h0, v2, e0, f1); //y
      h4->setNeighbors(h5, h9, v3, e4, f1); //y
      h5->setNeighbors(h3, h6, v1, e1, f1); //y
      h6->setNeighbors(h6->next(), h5, v2, e1, h6->face()); //y
      h7->setNeighbors(h7->next(), h1, v0, e2, h7->face()); //y
      h8->setNeighbors(h8->next(), h2, v3, e3, h8->face()); //y
      h9->setNeighbors(h9->next(), h4, v1, e4, h9->face()); //yes
      

      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3; 
      v3->halfedge() = h0; 

      e0->halfedge() = h0;
      e1->halfedge() = h5; 
      e2->halfedge() = h1; 
      e3->halfedge() = h2; 
      e4->halfedge() = h4; 

      f0->halfedge() = h0; 
      f1->halfedge() = h3; 

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {

      if (e0->isBoundary()) {
          return VertexIter();
      }
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      //New Elements
 
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();
      HalfedgeIter h13 = newHalfedge();
      HalfedgeIter h14 = newHalfedge();
      HalfedgeIter h15 = newHalfedge();


      VertexIter v4 = newVertex();

      EdgeIter e5 = newEdge();
      EdgeIter e6 = newEdge();
      EdgeIter e7 = newEdge();

      FaceIter f2 = newFace();
      FaceIter f3 = newFace();


      //Alter pointers: 
      h0->setNeighbors(h15, h3, v0, e0, f0); //y
      h1->setNeighbors(h13, h6, v1, e1, f3); //y
      h2->setNeighbors(h0, h7, v2, e2, f0); //y   
      h3->setNeighbors(h4, h0, v4, e0, f1); //y
      h4->setNeighbors(h14, h8, v0, e3, f1); //y
      h5->setNeighbors(h10, h9, v3, e4, f2); //y
      h6->setNeighbors(h6->next(), h1, v2, e1, h6->face()); 
      h7->setNeighbors(h7->next(), h2, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h4, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h5, v1, e4, h9->face()); 
      h10->setNeighbors(h12, h11, v1, e7, f2);//y
      h11->setNeighbors(h1, h10, v4, e7, f3);//y
      h12->setNeighbors(h5, h14, v4, e6, f2);//y
      h13->setNeighbors(h11, h15, v2, e5, f3);//y
      h14->setNeighbors(h3, h12, v3, e6, f1);//y
      h15->setNeighbors(h2, h13, v4, e5, f0);//y



      v0->halfedge() = h4;
      v1->halfedge() = h1;
      v2->halfedge() = h2;
      v3->halfedge() = h5;
      v4->halfedge() = h3; 

      v4->isNew = true;

      e0->halfedge() = h3;
      e1->halfedge() = h1;
      e2->halfedge() = h2;
      e3->halfedge() = h4;
      e4->halfedge() = h5;
      e5->halfedge() = h13;
      e6->halfedge() = h12;
      e7->halfedge() = h10;

      e5->isNew = true;
      e6->isNew = true;
      e7->isNew = false;


      f0->halfedge() = h0;
      f1->halfedge() = h3;
      f2->halfedge() = h10;
      f3->halfedge() = h11;

      v4->position = (v1->position + v0->position ) / 2; 
      //v4->computeCentroid();
      //v4->newPosition = v4->centroid;


    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.
      
    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.

      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          Vector3D original_neighbor_position_sum (0.0, 0.0,0.0); 
          Vector3D original_position = v->position;

          HalfedgeIter h = v->halfedge();      // get the outgoing half-edge of the vertex
          do {
              HalfedgeIter h_twin = h->twin(); // get the opposite half-edge
              VertexIter v_neighbor = h_twin->vertex(); 

              original_neighbor_position_sum += v_neighbor->position;

 
              h = h_twin->next();               // move to the next outgoing half-edge of the vertex
          } while (h != v->halfedge());          // keep going until we are back where we were

          float u;
          Size n = v->degree(); 

          if (n == 3) {
              u = 0.1875;//3/16
          }
          else {
              u = 3.0 / (8.0 * (float)n); 
          }
          
          v->newPosition = (1.0 - (float)n * u) * original_position + u * original_neighbor_position_sum;
          v->isNew = false; 

      }
      

    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {

          HalfedgeIter h = e->halfedge(); 
          HalfedgeIter h_twin = h->twin(); 

          VertexIter va = h->vertex();
          VertexIter vb = h_twin->vertex();



          VertexIter vc = h->next()->twin()->vertex();
          VertexIter vd = h_twin->next()->twin()->vertex();


          e->newPosition = 0.375 * (va->position + vb->position) + 0.125 * (vc->position + vd->position); 
          e->isNew = false;


      }

      //3/8 * (A + B) + 1/8 * (C + D)
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)



      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          
          if (e->isNew == false && e->halfedge()->vertex()->isNew == false && e->halfedge()->twin()->vertex()->isNew == false) {
              mesh.splitEdge(e)->newPosition = e->newPosition;
              
              //cout << "splitting ege \n";
          }

      }

    // 4. Flip any new edge that connects an old and new vertex.

      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          bool v1 = e->halfedge()->vertex()->isNew;
          bool v2 = e->halfedge()->twin()->vertex()->isNew;

          if (e->isNew == true && ((v1&&!v2)||(!v1&&v2)) ) {
              mesh.flipEdge(e);
              //cout << "splitting ege \n";
          }

      }
      
    // 5. Copy the new vertex positions into final Vertex::position.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->position = v->newPosition;
          

      }
  }
}
