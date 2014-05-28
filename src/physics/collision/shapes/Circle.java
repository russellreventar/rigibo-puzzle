package physics.collision.shapes;

import physics.collision.AABB;
import physics.tools.Mat22;
import physics.tools.Settings;
import physics.tools.Transform;
import physics.tools.Vec;

/**
 * A circle shape.
 */
public class Circle extends Shape {
	
	public final Vec m_p;
	
	private final Vec pool1 = new Vec();

	public Circle() {
		m_type = ShapeType.CIRCLE;
		m_p = new Vec();
		m_radius = 0;
	}
	
	public final Shape clone() {
		Circle shape = new Circle();
		shape.m_p.set(m_p);
		shape.m_radius = m_radius;
		return shape;
	}
	
	/**
	 * Get the supporting vertex index in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final int getSupport(final Vec d) {
		return 0;
	}
	
	/**
	 * Get the supporting vertex in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final Vec getSupportVertex(final Vec d) {
		return m_p;
	}
	
	/**
	 * Get the vertex count.
	 * 
	 * @return
	 */
	public final int getVertexCount() {
		return 1;
	}
	
	/**
	 * Get a vertex by index.
	 * 
	 * @param index
	 * @return
	 */
	public final Vec getVertex(final int index) {
		assert (index == 0);
		return m_p;
	}
	
	/**
	 * @see Shape#testPoint(Transform, Vec)
	 */
	@Override
	public final boolean testPoint(final Transform transform, final Vec p) {
		final Vec center = pool1;
		Mat22.mulToOut(transform.R, m_p, center);
		center.addLocal(transform.position);
		
		final Vec d = center.subLocal(p).negateLocal();
		return Vec.dot(d, d) <= m_radius * m_radius;
	}
		
	/**
	 * @see physics.collision.shapes.Shape#computeAABB(physics.collision.AABB,
	 *      physics.tools.Transform, int)
	 */
	@Override
	public final void computeAABB(final AABB argAabb, final Transform argTransform) {
		final Vec p = pool1;
		Mat22.mulToOut(argTransform.R, m_p, p);
		p.addLocal(argTransform.position);
		
		argAabb.lowerBound.x = p.x - m_radius;
		argAabb.lowerBound.y = p.y - m_radius;
		argAabb.upperBound.x = p.x + m_radius;
		argAabb.upperBound.y = p.y + m_radius;
	}
	
	/**
	 * @see Shape#computeMass(Mass, float)
	 */
	@Override
	public final void computeMass(final Mass massData, final float density) {
		massData.mass = density * Settings.PI * m_radius * m_radius;
		massData.center.set(m_p);
		
		// inertia about the local origin
		massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec.dot(m_p, m_p));
	}
}
