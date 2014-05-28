package physics.collision;

import physics.tools.Settings;
import physics.tools.Vec;

public class Manifold {
	
	public static enum ManifoldType {
		CIRCLES, FACE_A, FACE_B
	}
	
	/** The points of contact. */
	public final ManifoldPoint[] points;
	
	/** not use for Type::e_points */
	public final Vec localNormal;
	
	/** usage depends on manifold type */
	public final Vec localPoint;
	
	public ManifoldType type;
	
	/** The number of manifold points. */
	public int pointCount;
	
	/**
	 * creates a manifold with 0 points, with it's points array
	 * full of instantiated ManifoldPoints.
	 */
	public Manifold() {
		points = new ManifoldPoint[Settings.maxManifoldPoints];
		for (int i = 0; i < Settings.maxManifoldPoints; i++) {
			points[i] = new ManifoldPoint();
		}
		localNormal = new Vec();
		localPoint = new Vec();
		pointCount = 0;
	}
	
	/**
	 * Creates this manifold as a copy of the other
	 * 
	 * @param other
	 */
	public Manifold(Manifold other) {
		points = new ManifoldPoint[Settings.maxManifoldPoints];
		localNormal = other.localNormal.clone();
		localPoint = other.localPoint.clone();
		pointCount = other.pointCount;
		type = other.type;
		// djm: this is correct now
		for (int i = 0; i < Settings.maxManifoldPoints; i++) {
			points[i] = new ManifoldPoint(other.points[i]);
		}
	}
	
	/**
	 * copies this manifold from the given one
	 * 
	 * @param cp
	 *            manifold to copy from
	 */
	public void set(Manifold cp) {
		for (int i = 0; i < cp.pointCount; i++) {
			points[i].set(cp.points[i]);
		}
		
		type = cp.type;
		localNormal.set(cp.localNormal);
		localPoint.set(cp.localPoint);
		pointCount = cp.pointCount;
	}
}
