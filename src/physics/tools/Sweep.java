package physics.tools;

import java.io.Serializable;

/**
 * This describes the motion of a body/shape for TOI computation.
 * Shapes are defined with respect to the body origin, which may
 * no coincide with the center of mass. However, to support dynamics
 * we must interpolate the center of mass position.
 */
public class Sweep implements Serializable {
	private static final long serialVersionUID = 1L;
	
	/** Local center of mass position */
	public final Vec localCenter;
	/** Center world positions */
	public final Vec c0, c;
	/** World angles */
	public float a0, a;
	
	public String toString() {
		String s = "Sweep:\nlocalCenter: " + localCenter + "\n";
		s += "c0: " + c0 + ", c: " + c + "\n";
		s += "a0: " + a0 + ", a: " + a + "\n";
		return s;
	}
	
	public Sweep() {
		localCenter = new Vec();
		c0 = new Vec();
		c = new Vec();
	}
	
	public final void normalize() {
		float d = MathUtils.TWOPI * MathUtils.floor(a0 / MathUtils.TWOPI);
		a0 -= d;
		a -= d;
	}
	
	public final Sweep set(Sweep argCloneFrom) {
		localCenter.set(argCloneFrom.localCenter);
		c0.set(argCloneFrom.c0);
		c.set(argCloneFrom.c);
		a0 = argCloneFrom.a0;
		a = argCloneFrom.a;
		return this;
	}
	
	/**
	 * Get the interpolated transform at a specific time.
	 * 
	 * @param xf
	 *            the result is placed here - must not be null
	 * @param t
	 *            the normalized time in [0,1].
	 */
	public final void getTransform(final Transform xf, final float alpha) {
		assert (xf != null);

		xf.position.x = (1.0f - alpha) * c0.x + alpha * c.x;
		xf.position.y = (1.0f - alpha) * c0.y + alpha * c.y;
		// float angle = (1.0f - alpha) * a0 + alpha * a;
		// xf.R.set(angle);
		xf.R.set((1.0f - alpha) * a0 + alpha * a);
		
		// Shift to origin
		// xf.position.subLocal(Mat22.mul(xf.R, localCenter));
		xf.position.x -= xf.R.col1.x * localCenter.x + xf.R.col2.x * localCenter.y;
		xf.position.y -= xf.R.col1.y * localCenter.x + xf.R.col2.y * localCenter.y;
	}
	
	/**
	 * Advance the sweep forward, yielding a new initial state.
	 * 
	 * @param t
	 *            the new initial time.
	 */
	public final void advance(final float t) {
		// c0 = (1.0f - t) * c0 + t*c;
		c0.x = (1.0f - t) * c0.x + t * c.x;
		c0.y = (1.0f - t) * c0.y + t * c.y;
		a0 = (1.0f - t) * a0 + t * a;
	}
	
}
