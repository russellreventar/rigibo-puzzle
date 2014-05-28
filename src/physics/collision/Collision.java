package physics.collision;

import physics.collision.Distance.SimplexCache;
import physics.collision.Manifold.ManifoldType;
import physics.collision.shapes.Circle;
import physics.collision.shapes.Polygon;
import physics.collision.shapes.Shape;
import physics.dataStorage.IWorldPool;
import physics.tools.Mat22;
import physics.tools.Settings;
import physics.tools.Transform;
import physics.tools.Vec;

/**
 * Functions used for computing contact points, distance
 * queries, and TOI queries. 
 */
public class Collision {
	public static final int NULL_FEATURE = Integer.MAX_VALUE;
	
	private final IWorldPool pool;
	
	public Collision(IWorldPool argPool) {
		incidentEdge[0] = new ClipVertex();
		incidentEdge[1] = new ClipVertex();
		clipPoints1[0] = new ClipVertex();
		clipPoints1[1] = new ClipVertex();
		clipPoints2[0] = new ClipVertex();
		clipPoints2[1] = new ClipVertex();
		pool = argPool;
	}
	
	private final DistanceInput input = new DistanceInput();
	private final SimplexCache cache = new SimplexCache();
	private final DistanceOutput output = new DistanceOutput();
	
	/**
	 * Determine if two generic shapes overlap.
	 * 
	 * @param shapeA
	 * @param shapeB
	 * @param xfA
	 * @param xfB
	 * @return
	 */
	public final boolean testOverlap(Shape shapeA, Shape shapeB, Transform xfA, Transform xfB) {
		input.proxyA.set(shapeA);
		input.proxyB.set(shapeB);
		input.transformA.set(xfA);
		input.transformB.set(xfB);
		input.useRadii = true;
		
		cache.count = 0;
		
		pool.getDistance().distance(output, cache, input);
		// djm note: anything significant about 10.0f?
		return output.distance < 10.0f * Settings.EPSILON;
	}
	
	/**
	 * Compute the point states given two manifolds. The states pertain to the transition
	 * from manifold1
	 * to manifold2. So state1 is either persist or remove while state2 is either add or
	 * persist.
	 * 
	 * @param state1
	 * @param state2
	 * @param manifold1
	 * @param manifold2
	 */
	public static final void getPointStates(final PointState[] state1, final PointState[] state2,
			final Manifold manifold1, final Manifold manifold2) {
		
		for (int i = 0; i < Settings.maxManifoldPoints; i++) {
			state1[i] = PointState.NULL_STATE;
			state2[i] = PointState.NULL_STATE;
		}
		
		// Detect persists and removes.
		for (int i = 0; i < manifold1.pointCount; i++) {
			ContactID id = manifold1.points[i].id;
			
			state1[i] = PointState.REMOVE_STATE;
			
			for (int j = 0; j < manifold2.pointCount; j++) {
				if (manifold2.points[j].id.isEqual(id)) {
					state1[i] = PointState.PERSIST_STATE;
					break;
				}
			}
		}
		
		// Detect persists and adds
		for (int i = 0; i < manifold2.pointCount; i++) {
			ContactID id = manifold2.points[i].id;
			
			state2[i] = PointState.ADD_STATE;
			
			for (int j = 0; j < manifold1.pointCount; j++) {
				if (manifold1.points[j].id.isEqual(id)) {
					state2[i] = PointState.PERSIST_STATE;
					break;
				}
			}
		}
	}
	
	/**
	 * Clipping for contact manifolds.
	 * Sutherland-Hodgman clipping.
	 * 
	 * @param vOut
	 * @param vIn
	 * @param normal
	 * @param offset
	 * @return
	 */
	public static final int clipSegmentToLine(final ClipVertex[] vOut, final ClipVertex[] vIn, final Vec normal,
			float offset) {
		
		// Start with no output points
		int numOut = 0;
		
		// Calculate the distance of end points to the line
		float distance0 = Vec.dot(normal, vIn[0].v) - offset;
		float distance1 = Vec.dot(normal, vIn[1].v) - offset;
		
		// If the points are behind the plane
		if (distance0 <= 0.0f) {
			vOut[numOut++].set(vIn[0]);
		}
		if (distance1 <= 0.0f) {
			vOut[numOut++].set(vIn[1]);
		}
		
		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0f) {
			// Find intersection point of edge and plane
			float interp = distance0 / (distance0 - distance1);
			// vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
			vOut[numOut].v.set(vIn[1].v).subLocal(vIn[0].v).mulLocal(interp).addLocal(vIn[0].v);
			if (distance0 > 0.0f) {
				vOut[numOut].id.set(vIn[0].id);
			}
			else {
				vOut[numOut].id.set(vIn[1].id);
			}
			++numOut;
		}
		
		return numOut;
	}
	
	// #### COLLISION STUFF (not from collision.h or collision.cpp) ####
	
	// djm pooling
	
	/**
	 * Compute the collision manifold between two circles.
	 * 
	 * @param manifold
	 * @param circle1
	 * @param xfA
	 * @param circle2
	 * @param xfB
	 */
	public final void collideCircles(Manifold manifold, final Circle circle1, final Transform xfA,
			final Circle circle2, final Transform xfB) {
		manifold.pointCount = 0;
		
    // before inline:
//		Transform.mulToOut(xfA, circle1.m_p, pA);
//    Transform.mulToOut(xfB, circle2.m_p, pB);
//		d.set(pB).subLocal(pA);
//    float distSqr = d.x * d.x + d.y * d.y;
    
    // after inline:
    final Vec v = circle1.m_p;
		final float pAy = xfA.position.y + xfA.R.col1.y * v.x + xfA.R.col2.y * v.y;
    final float pAx = xfA.position.x + xfA.R.col1.x * v.x + xfA.R.col2.x * v.y;

    final Vec v1 = circle2.m_p;
		final float pBy = xfB.position.y + xfB.R.col1.y * v1.x + xfB.R.col2.y * v1.y;
    final float pBx = xfB.position.x + xfB.R.col1.x * v1.x + xfB.R.col2.x * v1.y;
    
    final float dx = pBx - pAx;
    final float dy = pBy - pAy;
    
    final float distSqr = dx * dx + dy * dy;
    // end inline
		
		final float radius = circle1.m_radius + circle2.m_radius;
		if (distSqr > radius * radius) {
			return;
		}
		
		manifold.type = ManifoldType.CIRCLES;
		manifold.localPoint.set(circle1.m_p);
		manifold.localNormal.setZero();
		manifold.pointCount = 1;
		
		manifold.points[0].localPoint.set(circle2.m_p);
		manifold.points[0].id.zero();
	}
	
	/**
	 * Compute the collision manifold between a polygon and a circle.
	 * 
	 * @param manifold
	 * @param polygon
	 * @param xfA
	 * @param circle
	 * @param xfB
	 */
	public final void collidePolygonAndCircle(Manifold manifold, final Polygon polygon, final Transform xfA,
			final Circle circle, final Transform xfB) {
		manifold.pointCount = 0;
    Vec v = circle.m_p;
		final float cy = xfB.position.y + xfB.R.col1.y * v.x + xfB.R.col2.y * v.y;
    final float cx = xfB.position.x + xfB.R.col1.x * v.x + xfB.R.col2.x * v.y;
		final float v1x = cx - xfA.position.x;
    final float v1y = cy - xfA.position.y;
    final Vec b = xfA.R.col1;
    final Vec b1 = xfA.R.col2;
    final float cLocaly = v1x * b1.x + v1y * b1.y;
    final float cLocalx = v1x * b.x + v1y * b.y;
    // end inline
		
		// Find the min separating edge.
		int normalIndex = 0;
		float separation = -Float.MAX_VALUE;
		final float radius = polygon.m_radius + circle.m_radius;
		final int vertexCount = polygon.m_vertexCount;
		
		final Vec[] vertices = polygon.m_vertices;
		final Vec[] normals = polygon.m_normals;
		
		for (int i = 0; i < vertexCount; i++) {
		  final Vec vertex = vertices[i];
		  final float tempx = cLocalx - vertex.x;
		  final float tempy = cLocaly - vertex.y;
		  final Vec normal = normals[i];
      final float s = normal.x * tempx + normal.y * tempy;
			
			
			if (s > radius) {
				// early out
				return;
			}
			
			if (s > separation) {
				separation = s;
				normalIndex = i;
			}
		}
		
		// Vertices that subtend the incident face.
		final int vertIndex1 = normalIndex;
		final int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
		final Vec v1 = vertices[vertIndex1];
		final Vec v2 = vertices[vertIndex2];
		
		// If the center is inside the polygon ...
		if (separation < Settings.EPSILON) {
			manifold.pointCount = 1;
			manifold.type = ManifoldType.FACE_A;
			
			final Vec normal = normals[normalIndex];
			manifold.localNormal.x = normal.x;
			manifold.localNormal.y = normal.y;
			manifold.localPoint.x = (v1.x + v2.x) * .5f;
			manifold.localPoint.y = (v1.y + v2.y) * .5f;
			final ManifoldPoint mpoint = manifold.points[0];
			mpoint.localPoint.x = circle.m_p.x;
	    mpoint.localPoint.y = circle.m_p.y;
	    mpoint.id.zero();
	    // end inline
			return;
		}

    final float tempX = cLocalx - v1.x;
    final float tempY = cLocaly - v1.y;
    final float temp2X = v2.x - v1.x;
    final float temp2Y = v2.y - v1.y;
		final float u1 = tempX * temp2X + tempY * temp2Y;
		
		final float temp3X = cLocalx - v2.x;
		final float temp3Y = cLocaly - v2.y;
		final float temp4X = v1.x - v2.x;
		final float temp4Y = v1.y - v2.y;
		final float u2 = temp3X * temp4X + temp3Y * temp4Y;
		// end inline
		
		if (u1 <= 0f) {
		  // inlined
		  final float dx = cLocalx - v1.x;
		  final float dy = cLocaly - v1.y;
			if ( dx * dx + dy * dy > radius * radius) {
				return;
			}
			
			manifold.pointCount = 1;
			manifold.type = ManifoldType.FACE_A;

			manifold.localNormal.x = cLocalx - v1.x;
			manifold.localNormal.y = cLocaly - v1.y;
			//end inline
			manifold.localNormal.normalize();
			manifold.localPoint.set(v1);
			manifold.points[0].localPoint.set(circle.m_p);
			manifold.points[0].id.zero();
		}
		else if (u2 <= 0.0f) {
		  // inlined
      final float dx = cLocalx - v2.x;
      final float dy = cLocaly - v2.y;
      if ( dx * dx + dy * dy > radius * radius) {
        return;
      }
			
			manifold.pointCount = 1;
			manifold.type = ManifoldType.FACE_A;
      // before inline:

      // after inline:
      manifold.localNormal.x = cLocalx - v2.x;
      manifold.localNormal.y = cLocaly - v2.y;
      //end inline
			manifold.localNormal.normalize();
			manifold.localPoint.set(v2);
			manifold.points[0].localPoint.set(circle.m_p);
			manifold.points[0].id.zero();
		}
		else {

			final float fcx = (v1.x + v2.x) * .5f;
			final float fcy = (v1.y + v2.y) * .5f;
			
			final float tx = cLocalx - fcx;
			final float ty = cLocaly - fcy;
			final Vec normal = normals[vertIndex1];
			separation = tx * normal.x + ty * normal.y;
			if(separation > radius){
			  return;
			}
			// end inline
			
			manifold.pointCount = 1;
			manifold.type = ManifoldType.FACE_A;
			manifold.localNormal.set(normals[vertIndex1]);
			manifold.localPoint.x = fcx; // (faceCenter)
			manifold.localPoint.y = fcy;
			manifold.points[0].localPoint.set(circle.m_p);
			manifold.points[0].id.zero();
		}
	}
	
	// djm pooling
	private final Vec normal1 = new Vec();
	
	/**
	 * Find the separation between poly1 and poly2 for a given edge normal on poly1.
	 * 
	 * @param poly1
	 * @param xf1
	 * @param edge1
	 * @param poly2
	 * @param xf2
	 */
	public final float edgeSeparation(final Polygon poly1, final Transform xf1, final int edge1,
			final Polygon poly2, final Transform xf2) {
		
		final int count1 = poly1.m_vertexCount;
		final Vec[] vertices1 = poly1.m_vertices;
		final Vec[] normals1 = poly1.m_normals;
		
		final int count2 = poly2.m_vertexCount;
		final Vec[] vertices2 = poly2.m_vertices;
		
		assert (0 <= edge1 && edge1 < count1);

    final Mat22 R = xf1.R;
    final Vec v = normals1[edge1];
    final float normal1Worldy = R.col1.y * v.x + R.col2.y * v.y;
    final float normal1Worldx = R.col1.x * v.x + R.col2.x * v.y;
    final Mat22 R1 = xf2.R;
    final float normal1x = normal1Worldx * R1.col1.x + normal1Worldy * R1.col1.y;
    final float normal1y = normal1Worldx * R1.col2.x + normal1Worldy * R1.col2.y;
    // end inline
		
		// Find support vertex on poly2 for -normal.
		int index = 0;
		float minDot = Float.MAX_VALUE;
		
		for (int i = 0; i < count2; ++i) {
			final Vec a = vertices2[i];
      final float dot = a.x * normal1x + a.y * normal1y;
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}

    final Vec v3 = vertices1[edge1];
    final float v1y = xf1.position.y + R.col1.y * v3.x + R.col2.y * v3.y;
    final float v1x = xf1.position.x + R.col1.x * v3.x + R.col2.x * v3.y;
    final Vec v4 = vertices2[index];
    final float v2y = xf2.position.y + R1.col1.y * v4.x + R1.col2.y * v4.y - v1y;
    final float v2x = xf2.position.x + R1.col1.x * v4.x + R1.col2.x * v4.y - v1x;
        
		return v2x * normal1Worldx + v2y * normal1Worldy;
		// end inline
	}
	
	// djm pooling, and from above
	
	/**
	 * Find the max separation between poly1 and poly2 using edge normals from poly1.
	 * 
	 * @param edgeIndex
	 * @param poly1
	 * @param xf1
	 * @param poly2
	 * @param xf2
	 * @return
	 */
	public final void findMaxSeparation(EdgeResults results, final Polygon poly1, final Transform xf1,
			final Polygon poly2, final Transform xf2) {
		int count1 = poly1.m_vertexCount;
		final Vec[] normals1 = poly1.m_normals;
    Vec v = poly2.m_centroid;
		
		final float predy = xf2.position.y + xf2.R.col1.y * v.x + xf2.R.col2.y * v.y;
    final float predx = xf2.position.x + xf2.R.col1.x * v.x + xf2.R.col2.x * v.y;
    final Vec v1 = poly1.m_centroid;
    final float tempy = xf1.position.y + xf1.R.col1.y * v1.x + xf1.R.col2.y * v1.y;
    final float tempx = xf1.position.x + xf1.R.col1.x * v1.x + xf1.R.col2.x * v1.y;
    final float dx = predx - tempx;
    final float dy = predy - tempy;
    
    final Mat22 R = xf1.R;
    final float dLocal1x = dx * R.col1.x + dy * R.col1.y;
    final float dLocal1y = dx * R.col2.x + dy * R.col2.y;

		int edge = 0;
		float dot;
		float maxDot = -Float.MAX_VALUE;
		for (int i = 0; i < count1; i++) {
		  final Vec normal = normals1[i];
			dot = normal.x * dLocal1x + normal.y * dLocal1y;
			if (dot > maxDot) {
				maxDot = dot;
				edge = i;
			}
		}
		
		// Get the separation for the edge normal.
		float s = edgeSeparation(poly1, xf1, edge, poly2, xf2);
		
		// Check the separation for the previous edge normal.
		int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
		float sPrev = edgeSeparation(poly1, xf1, prevEdge, poly2, xf2);
		
		// Check the separation for the next edge normal.
		int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
		float sNext = edgeSeparation(poly1, xf1, nextEdge, poly2, xf2);
		
		// Find the best edge and the search direction.
		int bestEdge;
		float bestSeparation;
		int increment;
		if (sPrev > s && sPrev > sNext) {
			increment = -1;
			bestEdge = prevEdge;
			bestSeparation = sPrev;
		}
		else if (sNext > s) {
			increment = 1;
			bestEdge = nextEdge;
			bestSeparation = sNext;
		}
		else {
			results.edgeIndex = edge;
			results.separation = s;
			return;
		}
		
		// Perform a local search for the best edge normal.
		for (;;) {
			if (increment == -1) {
				edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
			}
			else {
				edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
			}
			
			s = edgeSeparation(poly1, xf1, edge, poly2, xf2);
			
			if (s > bestSeparation) {
				bestEdge = edge;
				bestSeparation = s;
			}
			else {
				break;
			}
		}
		
		results.edgeIndex = bestEdge;
		results.separation = bestSeparation;
	}
	
	// djm pooling from above
	public final void findIncidentEdge(final ClipVertex[] c, final Polygon poly1, final Transform xf1, int edge1,
			final Polygon poly2, final Transform xf2) {
		int count1 = poly1.m_vertexCount;
		final Vec[] normals1 = poly1.m_normals;
		
		int count2 = poly2.m_vertexCount;
		final Vec[] vertices2 = poly2.m_vertices;
		final Vec[] normals2 = poly2.m_normals;
		
		assert (0 <= edge1 && edge1 < count1);
		
		// Get the normal of the reference edge in poly2's frame.
		Mat22.mulToOut(xf1.R, normals1[edge1], normal1); // temporary
		Mat22.mulTransToOut(xf2.R, normal1, normal1);
		
		// Find the incident edge on poly2.
		int index = 0;
		float minDot = Float.MAX_VALUE;
		for (int i = 0; i < count2; ++i) {
			float dot = Vec.dot(normal1, normals2[i]);
			if (dot < minDot) {
				minDot = dot;
				index = i;
			}
		}
		
		// Build the clip vertices for the incident edge.
		int i1 = index;
		int i2 = i1 + 1 < count2 ? i1 + 1 : 0;
		
		Transform.mulToOut(xf2, vertices2[i1], c[0].v); // = Mul(xf2, vertices2[i1]);
		c[0].id.features.referenceEdge = edge1;
		c[0].id.features.incidentEdge = i1;
		c[0].id.features.incidentVertex = 0;
		
		Transform.mulToOut(xf2, vertices2[i2], c[1].v); // = Mul(xf2, vertices2[i2]);
		c[1].id.features.referenceEdge = edge1;
		c[1].id.features.incidentEdge = i2;
		c[1].id.features.incidentVertex = 1;
	}
	
	private final EdgeResults results1 = new EdgeResults();
	private final EdgeResults results2 = new EdgeResults();
	private final ClipVertex[] incidentEdge = new ClipVertex[2];
	private final Vec localTangent = new Vec();
	private final Vec localNormal = new Vec();
	private final Vec planePoint = new Vec();
	private final Vec tangent = new Vec();
	private final Vec normal = new Vec();
	private final Vec v11 = new Vec();
	private final Vec v12 = new Vec();
	private final ClipVertex[] clipPoints1 = new ClipVertex[2];
	private final ClipVertex[] clipPoints2 = new ClipVertex[2];
	
	/**
	 * Compute the collision manifold between two polygons.
	 * 
	 * @param manifold
	 * @param polygon1
	 * @param xf1
	 * @param polygon2
	 * @param xf2
	 */
	public final void collidePolygons(Manifold manifold, final Polygon polyA, final Transform xfA,
			final Polygon polyB, final Transform xfB) {
		// Find edge normal of max separation on A - return if separating axis is found
		// Find edge normal of max separation on B - return if separation axis is found
		// Choose reference edge as min(minA, minB)
		// Find incident edge
		// Clip
		
		// The normal points from 1 to 2
		
		manifold.pointCount = 0;
		float totalRadius = polyA.m_radius + polyB.m_radius;
		
		findMaxSeparation(results1, polyA, xfA, polyB, xfB);
		if (results1.separation > totalRadius) {
			return;
		}
		
		findMaxSeparation(results2, polyB, xfB, polyA, xfA);
		if (results2.separation > totalRadius) {
			return;
		}
		
		final Polygon poly1; // reference polygon
		final Polygon poly2; // incident polygon
		Transform xf1, xf2;
		int edge1; // reference edge
		int flip;
		final float k_relativeTol = 0.98f;
		final float k_absoluteTol = 0.001f;
		
		if (results2.separation > k_relativeTol * results1.separation + k_absoluteTol) {
			poly1 = polyB;
			poly2 = polyA;
			xf1 = xfB;
			xf2 = xfA;
			edge1 = results2.edgeIndex;
			manifold.type = ManifoldType.FACE_B;
			flip = 1;
		}
		else {
			poly1 = polyA;
			poly2 = polyB;
			xf1 = xfA;
			xf2 = xfB;
			edge1 = results1.edgeIndex;
			manifold.type = ManifoldType.FACE_A;
			flip = 0;
		}
		
		findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);
		
		int count1 = poly1.m_vertexCount;
		final Vec[] vertices1 = poly1.m_vertices;
		
		v11.set(vertices1[edge1]);
		v12.set(edge1 + 1 < count1 ? vertices1[edge1 + 1] : vertices1[0]);
		
		localTangent.set(v12).subLocal(v11);
		localTangent.normalize();
		
		Vec.crossToOut(localTangent, 1f, localNormal); // Vec2 localNormal = Cross(dv,
														// 1.0f);
		
		planePoint.set(v11).addLocal(v12).mulLocal(.5f); // Vec2 planePoint = 0.5f * (v11
															// + v12);
		
		Mat22.mulToOut(xf1.R, localTangent, tangent); // Vec2 sideNormal = Mul(xf1.R, v12
														// - v11);
		Vec.crossToOut(tangent, 1f, normal); // Vec2 frontNormal = Cross(sideNormal,
												// 1.0f);
		
		Transform.mulToOut(xf1, v11, v11);
		Transform.mulToOut(xf1, v12, v12);
		// v11 = Mul(xf1, v11);
		// v12 = Mul(xf1, v12);
		
		// Face offset
		float frontOffset = Vec.dot(normal, v11);
		
		// Side offsets, extended by polytope skin thickness.
		float sideOffset1 = -Vec.dot(tangent, v11) + totalRadius;
		float sideOffset2 = Vec.dot(tangent, v12) + totalRadius;
		
		// Clip incident edge against extruded edge1 side edges.
		// ClipVertex clipPoints1[2];
		// ClipVertex clipPoints2[2];
		int np;
		
		// Clip to box side 1
		// np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);
		tangent.negateLocal();
		np = clipSegmentToLine(clipPoints1, incidentEdge, tangent, sideOffset1);
		tangent.negateLocal();
		
		if (np < 2) {
			return;
		}
		
		// Clip to negative box side 1
		np = clipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2);
		
		if (np < 2) {
			return;
		}
		
		// Now clipPoints2 contains the clipped points.
		manifold.localNormal.set(localNormal);
		manifold.localPoint.set(planePoint);
		
		int pointCount = 0;
		for (int i = 0; i < Settings.maxManifoldPoints; ++i) {
			float separation = Vec.dot(normal, clipPoints2[i].v) - frontOffset;
			
			if (separation <= totalRadius) {
				ManifoldPoint cp = manifold.points[pointCount];
				Transform.mulTransToOut(xf2, clipPoints2[i].v, cp.localPoint);
				// cp.m_localPoint = MulT(xf2, clipPoints2[i].v);
				cp.id.set(clipPoints2[i].id);
				cp.id.features.flip = flip;
				++pointCount;
			}
		}
		
		manifold.pointCount = pointCount;
	}
	
	/**
	 * Java-specific class for returning edge results
	 */
	private static class EdgeResults {
		public float separation;
		public int edgeIndex;
	}
	
	/**
	 * Used for computing contact manifolds.
	 */
	public static class ClipVertex{
		public final Vec v;
		public final ContactID id;

		public ClipVertex(){
			v = new Vec();
			id = new ContactID();
		}

		public void set(final ClipVertex cv){
			v.set(cv.v);
			id.set(cv.id);
		}
	}
	
	/**
	 * This is used for determining the state of contact points.
	 * @author Daniel Murphy
	 */
	public static enum PointState {
		/**
		 * point does not exist
		 */
		NULL_STATE,
		/**
		 * point was added in the update
		 */
		ADD_STATE,
		/**
		 * point persisted across the update
		 */
		PERSIST_STATE,
		/**
		 * point was removed in the update
		 */
		REMOVE_STATE
	}

}
