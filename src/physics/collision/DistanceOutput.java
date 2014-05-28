package physics.collision;

import physics.tools.Vec;

/**
 * Output for Distance.
 */
public class DistanceOutput {
	/** Closest point on shapeA */
	public final Vec pointA = new Vec();
	
	/** Closest point on shapeB */
	public final Vec pointB = new Vec();
	
	public float distance;
	
	/** number of gjk iterations used */
	public int iterations;
}
