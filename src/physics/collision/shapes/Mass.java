package physics.collision.shapes;

import physics.tools.Vec;

/** This holds the mass data computed for a shape. */
public class Mass {
	/** The mass of the shape, usually in kilograms. */
	public float mass;
	/** The position of the shape's centroid relative to the shape's origin. */
	public final Vec center;
	/** The rotational inertia of the shape about the local origin. */
	public float I;
	
	/**
	 * Blank mass data
	 */
	public Mass() {
		mass = I = 0f;
		center = new Vec();
	}
	
	/**
	 * Copies from the given mass data
	 * 
	 * @param md
	 *            mass data to copy from
	 */
	public Mass(Mass md) {
		mass = md.mass;
		I = md.I;
		center = md.center.clone();
	}
	
	public void set(Mass md) {
		mass = md.mass;
		I = md.I;
		center.set(md.center);
	}
}
