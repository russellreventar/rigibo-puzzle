package physics.collision.shapes;


import physics.collision.AABB;
import physics.tools.Transform;
import physics.tools.Vec;


/**
 * A shape is used for collision detection. You can create a shape however you like.
 * Shapes used for simulation in World are created automatically when a Fixture
 * is created.  Shapes may encapsulate a one or more child shapes.
 */
public abstract class Shape {

	public ShapeType m_type;
	public float m_radius;

	public Shape() {
		m_type = ShapeType.UNKNOWN;
	}
	
	/**
	 * Get the type of this shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public ShapeType getType() {
		return m_type;
	}

	/**
	 * Test a point for containment in this shape. This only works for convex shapes.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 */
	public abstract boolean testPoint( final Transform xf, final Vec p);

	/**
	 * Given a transform, compute the associated axis aligned bounding box for a child shape.
	 * @param argAabb returns the axis aligned box.
	 * @param argXf the world transform of the shape.
	 */
	public abstract void computeAABB(final AABB argAabb, final Transform argXf);

	/**
	 * Compute the mass properties of this shape using its dimensions and density.
	 * The inertia tensor is computed about the local origin.
	 * @param massData returns the mass data for this shape.
	 * @param density the density in kilograms per meter squared.
	 */
	public abstract void computeMass(final Mass massData, final float density);
		
	public abstract Shape clone();
}
