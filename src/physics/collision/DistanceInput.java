package physics.collision;


import physics.collision.Distance.DistanceProxy;
import physics.tools.Transform;

/**
 * Input for Distance.
 * You have to option to use the shape radii
 * in the computation.
 */
public class DistanceInput {
	public DistanceProxy proxyA = new DistanceProxy();
	public DistanceProxy proxyB = new DistanceProxy();
	public Transform transformA = new Transform();
	public Transform transformB = new Transform();
	public boolean useRadii;
}
