package physics.collision.shapes;


import physics.collision.AABB;
import physics.tools.Mat22;
import physics.tools.Settings;
import physics.tools.Transform;
import physics.tools.Vec;

/**
 * A polygon shape
 */
public class Polygon extends Shape {
	/** Dump lots of debug information. */
	private static boolean m_debug = false;

	/**
	 * Local position of the shape centroid in parent body frame.
	 */
	public final Vec m_centroid = new Vec();

	/**
	 * The vertices of the shape. Note: use getVertexCount(), not
	 * m_vertices.length, to get number of active vertices.
	 */
	public final Vec m_vertices[];

	/**
	 * The normals of the shape. Note: use getVertexCount(), not
	 * m_normals.length, to get number of active normals.
	 */
	public final Vec m_normals[];

	/**
	 * Number of active vertices in the shape.
	 */
	public int m_vertexCount;

	// pooling
	private final Vec pool1 = new Vec();
	private final Vec pool2 = new Vec();
	private final Vec pool3 = new Vec();
	private final Vec pool4 = new Vec();
	private Transform poolt1 = new Transform();

	public Polygon() {
		m_type = ShapeType.POLYGON;

		m_vertexCount = 0;
		m_vertices = new Vec[Settings.maxPolygonVertices];
		for (int i = 0; i < m_vertices.length; i++) {
			m_vertices[i] = new Vec();
		}
		m_normals = new Vec[Settings.maxPolygonVertices];
		for (int i = 0; i < m_normals.length; i++) {
			m_normals[i] = new Vec();
		}
		m_radius = Settings.polygonRadius;
		m_centroid.setZero();
	}

	public final Shape clone() {
		Polygon shape = new Polygon();
		shape.m_centroid.set(this.m_centroid);
		for (int i = 0; i < shape.m_normals.length; i++) {
			shape.m_normals[i].set(m_normals[i]);
			shape.m_vertices[i].set(m_vertices[i]);
		}
		shape.m_radius = this.m_radius;
		shape.m_vertexCount = this.m_vertexCount;
		return shape;
	}

	/**
	 * Get the supporting vertex index in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final int getSupport(final Vec d) {
		int bestIndex = 0;
		float bestValue = Vec.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; i++) {
			float value = Vec.dot(m_vertices[i], d);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return bestIndex;
	}

	/**
	 * Get the supporting vertex in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final Vec getSupportVertex(final Vec d) {
		int bestIndex = 0;
		float bestValue = Vec.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; i++) {
			float value = Vec.dot(m_vertices[i], d);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return m_vertices[bestIndex];
	}

	/**
	 * Copy vertices. This assumes the vertices define a convex polygon. It is
	 * assumed that the exterior is the the right of each edge.
	 */
	public final void set(final Vec[] vertices, final int count) {
		assert (2 <= count && count <= Settings.maxPolygonVertices);
		m_vertexCount = count;

		// Copy vertices.
		for (int i = 0; i < m_vertexCount; ++i) {
			if (m_vertices[i] == null) {
				m_vertices[i] = new Vec();
			}
			m_vertices[i].set(vertices[i]);
		}

		final Vec edge = pool1;

		// Compute normals. Ensure the edges have non-zero length.
		for (int i = 0; i < m_vertexCount; ++i) {
			final int i1 = i;
			final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			edge.set(m_vertices[i2]).subLocal(m_vertices[i1]);

			assert (edge.lengthSquared() > Settings.EPSILON * Settings.EPSILON);
			Vec.crossToOut(edge, 1f, m_normals[i]);
			m_normals[i].normalize();
		}

		if (m_debug) {

			final Vec r = pool2;

			// Ensure the polygon is convex and the interior
			// is to the left of each edge.
			for (int i = 0; i < m_vertexCount; ++i) {
				final int i1 = i;
				final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
				edge.set(m_vertices[i2]).subLocal(m_vertices[i1]);

				for (int j = 0; j < m_vertexCount; ++j) {
					// Don't check vertices on the current edge.
					if (j == i1 || j == i2) {
						continue;
					}

					r.set(m_vertices[j]).subLocal(m_vertices[i1]);

					// Your polygon is non-convex (it has an indentation) or
					// has colinear edges.
					final float s = Vec.cross(edge, r);
					assert (s > 0.0f);
				}
			}
		}

		// Compute the polygon centroid.
		computeCentroidToOut(m_vertices, m_vertexCount, m_centroid);
	}

	/**
	 * Build vertices to represent an axis-aligned box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 */
	public final void setAsBox(final float hx, final float hy) {
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set(hx, -hy);
		m_vertices[2].set(hx, hy);
		m_vertices[3].set(-hx, hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.setZero();
	}

	/**
	 * Build vertices to represent an oriented box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 * @param center
	 *            the center of the box in local coordinates.
	 * @param angle
	 *            the rotation of the box in local coordinates.
	 */
	public final void setAsBox(final float hx, final float hy,
			final Vec center, final float angle) {
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set(hx, -hy);
		m_vertices[2].set(hx, hy);
		m_vertices[3].set(-hx, hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.set(center);

		final Transform xf = poolt1;
		xf.position.set(center);
		xf.R.set(angle);

		// Transform vertices and normals.
		for (int i = 0; i < m_vertexCount; ++i) {
			Transform.mulToOut(xf, m_vertices[i], m_vertices[i]);
			Mat22.mulToOut(xf.R, m_normals[i], m_normals[i]);
		}
	}

	/**
	 * Set this as a single edge.
	 * 
	 * @param v1
	 * @param v2
	 */
	public final void setAsEdge(final Vec v1, final Vec v2) {
		m_vertexCount = 2;
		m_vertices[0].set(v1);
		m_vertices[1].set(v2);
		m_centroid.set(v1).addLocal(v2).mulLocal(0.5f);
		// = 0.5f * (v1 + v2);
		m_normals[0].set(v2).subLocal(v1);
		Vec.crossToOut(m_normals[0], 1f, m_normals[0]);
		// m_normals[0] = Cross(v2 - v1, 1.0f);
		m_normals[0].normalize();
		m_normals[1].set(m_normals[0]).negateLocal();
	}

	/**
	 * @see Shape#testPoint(Transform, Vec)
	 */
	@Override
	public final boolean testPoint(final Transform xf, final Vec p) {

		final Vec pLocal = pool1;

		pLocal.set(p).subLocal(xf.position);
		Mat22.mulTransToOut(xf.R, pLocal, pLocal);

		if (m_debug) {
			System.out.println("--testPoint debug--");
			System.out.println("Vertices: ");
			for (int i = 0; i < m_vertexCount; ++i) {
				System.out.println(m_vertices[i]);
			}
			System.out.println("pLocal: " + pLocal);
		}

		final Vec temp = pool2;

		for (int i = 0; i < m_vertexCount; ++i) {
			temp.set(pLocal).subLocal(m_vertices[i]);
			final float dot = Vec.dot(m_normals[i], temp);
			if (dot > 0.0f) {
				return false;
			}
		}

		return true;
	}

	/**
	 * @see Shape#computeAABB(AABB, Transform, int)
	 */
	@Override
	public final void computeAABB(final AABB argAabb, final Transform argXf) {

		final Vec lower = pool1;
		final Vec upper = pool2;
		final Vec v = pool3;

		Transform.mulToOut(argXf, m_vertices[0], lower);
		upper.set(lower);

		for (int i = 1; i < m_vertexCount; ++i) {
			Transform.mulToOut(argXf, m_vertices[i], v);
			// Vec2 v = Mul(xf, m_vertices[i]);
			Vec.minToOut(lower, v, lower);
			Vec.maxToOut(upper, v, upper);
		}

		// Vec2 r(m_radius, m_radius);
		// aabb->lowerBound = lower - r;
		// aabb->upperBound = upper + r;

		argAabb.lowerBound.x = lower.x - m_radius;
		argAabb.lowerBound.y = lower.y - m_radius;
		argAabb.upperBound.x = upper.x + m_radius;
		argAabb.upperBound.y = upper.y + m_radius;
	}

	/**
	 * Get the vertex count.
	 * 
	 * @return
	 */
	public final int getVertexCount() {
		return m_vertexCount;
	}

	/**
	 * Get a vertex by index.
	 * 
	 * @param index
	 * @return
	 */
	public final Vec getVertex(final int index) {
		assert (0 <= index && index < m_vertexCount);
		return m_vertices[index];
	}

	public final void computeCentroidToOut(final Vec[] vs, final int count,
			final Vec out) {
		assert (count >= 3);

		out.set(0.0f, 0.0f);
		float area = 0.0f;

		if (count == 2) {
			out.set(vs[0]).addLocal(vs[1]).mulLocal(.5f);
			return;
		}

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vec pRef = pool1;
		pRef.setZero();

		final Vec e1 = pool2;
		final Vec e2 = pool3;

		final float inv3 = 1.0f / 3.0f;

		for (int i = 0; i < count; ++i) {
			// Triangle vertices.
			final Vec p1 = pRef;
			final Vec p2 = vs[i];
			final Vec p3 = i + 1 < count ? vs[i + 1] : vs[0];

			e1.set(p2).subLocal(p1);
			e2.set(p3).subLocal(p1);

			final float D = Vec.cross(e1, e2);

			final float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			e1.set(p1).addLocal(p2).addLocal(p3).mulLocal(triangleArea * inv3);
			out.addLocal(e1);
		}

		// Centroid
		assert (area > Settings.EPSILON);
		out.mulLocal(1.0f / area);
	}

	/**
	 * @see Shape#computeMass(Mass)
	 */
	public void computeMass(final Mass massData, float density) {

		assert (m_vertexCount >= 2);

		// A line segment has zero mass.
		if (m_vertexCount == 2) {
			// massData.center = 0.5f * (m_vertices[0] + m_vertices[1]);
			massData.center.set(m_vertices[0]).addLocal(m_vertices[1])
					.mulLocal(0.5f);
			massData.mass = 0.0f;
			massData.I = 0.0f;
			return;
		}

		final Vec center = pool1;
		center.setZero();
		float area = 0.0f;
		float I = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vec pRef = pool2;
		pRef.setZero();

		final float k_inv3 = 1.0f / 3.0f;

		final Vec e1 = pool3;
		final Vec e2 = pool4;

		for (int i = 0; i < m_vertexCount; ++i) {
			// Triangle vertices.
			final Vec p1 = pRef;
			final Vec p2 = m_vertices[i];
			final Vec p3 = i + 1 < m_vertexCount ? m_vertices[i + 1]
					: m_vertices[0];

			e1.set(p2);
			e1.subLocal(p1);

			e2.set(p3);
			e2.subLocal(p1);

			final float D = Vec.cross(e1, e2);

			final float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			center.x += triangleArea * k_inv3 * (p1.x + p2.x + p3.x);
			center.y += triangleArea * k_inv3 * (p1.y + p2.y + p3.y);

			final float px = p1.x, py = p1.y;
			final float ex1 = e1.x, ey1 = e1.y;
			final float ex2 = e2.x, ey2 = e2.y;

			final float intx2 = k_inv3
					* (0.25f * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px
							* ex2)) + 0.5f * px * px;
			final float inty2 = k_inv3
					* (0.25f * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py
							* ey2)) + 0.5f * py * py;

			I += D * (intx2 + inty2);
		}

		// Total mass
		massData.mass = density * area;

		// Center of mass
		assert (area > Settings.EPSILON);
		center.mulLocal(1.0f / area);
		massData.center.set(center);

		// Inertia tensor relative to the local origin.
		massData.I = I * density;
	}

	/*
	 * Get the local centroid relative to the parent body. / public Vec2
	 * getCentroid() { return m_centroid.clone(); }
	 */

	/** Get the vertices in local coordinates. */
	public Vec[] getVertices() {
		return m_vertices;
	}

	/** Get the edge normal vectors. There is one for each vertex. */
	public Vec[] getNormals() {
		return m_normals;
	}

	/** Get the centroid and apply the supplied transform. */
	public Vec centroid(final Transform xf) {
		return Transform.mul(xf, m_centroid);
	}

	/** Get the centroid and apply the supplied transform. */
	public Vec centroidToOut(final Transform xf, final Vec out) {
		Transform.mulToOut(xf, m_centroid, out);
		return out;
	}
}
