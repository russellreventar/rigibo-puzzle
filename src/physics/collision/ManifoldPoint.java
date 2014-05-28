package physics.collision;

import physics.tools.Vec;

public class ManifoldPoint {
	/** usage depends on manifold type */
	public final Vec localPoint;
	/** the non-penetration impulse */
	public float normalImpulse;
	/** the friction impulse */
	public float tangentImpulse;
	/** uniquely identifies a contact point between two shapes */
	public final ContactID id;

	/**
	 * Blank manifold point with everything zeroed out.
	 */
	public ManifoldPoint() {
		localPoint = new Vec();
		normalImpulse = tangentImpulse = 0f;
		id = new ContactID();
	}

	/**
	 * Creates a manifold point as a copy of the given point
	 * @param cp point to copy from
	 */
	public ManifoldPoint(final ManifoldPoint cp) {
		localPoint = cp.localPoint.clone();
		normalImpulse = cp.normalImpulse;
		tangentImpulse = cp.tangentImpulse;
		id = new ContactID(cp.id);
	}

	/**
	 * Sets this manifold point form the given one
	 * @param cp the point to copy from
	 */
	public void set(final ManifoldPoint cp){
		localPoint.set(cp.localPoint);
		normalImpulse = cp.normalImpulse;
		tangentImpulse = cp.tangentImpulse;
		id.set(cp.id);
	}
}
