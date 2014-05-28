package physics.tools;

import java.io.Serializable;

/**
 * A transform contains translation and rotation. It is used to represent
 * the position and orientation of rigid frames.
 */
public class Transform implements Serializable {
	private static final long serialVersionUID = 1L;

	/** The translation caused by the transform */
	public final Vec position;
	
	/** A matrix representing a rotation */
	public final Mat22 R;
	
	/** The default constructor. */
	public Transform() {
		position = new Vec();
		R = new Mat22();
	}
	
	/** Initialize as a copy of another transform. */
	public Transform(final Transform xf) {
		position = xf.position.clone();
		R = xf.R.clone();
	}
	
	/** Initialize using a position vector and a rotation matrix. */
	public Transform(final Vec _position, final Mat22 _R) {
		position = _position.clone();
		R = _R.clone();
	}
	
	/** Set this to equal another transform. */
	public final Transform set(final Transform xf) {
		position.set(xf.position);
		R.set(xf.R);
		return this;
	}
	
	/**
	 * Set this based on the position and angle.
	 * 
	 * @param p
	 * @param angle
	 */
	public final void set(Vec p, float angle) {
		position.set(p);
		R.set(angle);
	}
	
	/**
	 * Calculate the angle that the rotation matrix represents.
	 */
	public final float getAngle() {
		return MathUtils.atan2(R.col1.y, R.col1.x);
	}
	
	/** Set this to the identity transform. */
	public final void setIdentity() {
		position.setZero();
		R.setIdentity();
	}
	
	public final static Vec mul(final Transform T, final Vec v) {
		return new Vec(T.position.x + T.R.col1.x * v.x + T.R.col2.x * v.y, T.position.y + T.R.col1.y * v.x
				+ T.R.col2.y * v.y);
	}
	
	/* djm added */
	public final static void mulToOut(final Transform T, final Vec v, final Vec out) {
		final float tempy = T.position.y + T.R.col1.y * v.x + T.R.col2.y * v.y;
		out.x = T.position.x + T.R.col1.x * v.x + T.R.col2.x * v.y;
		out.y = tempy;
	}
	
	public final static Vec mulTrans(final Transform T, final Vec v) {
		final float v1x = v.x - T.position.x;
		final float v1y = v.y - T.position.y;
		final Vec b = T.R.col1;
		final Vec b1 = T.R.col2;
		return new Vec((v1x * b.x + v1y * b.y), (v1x * b1.x + v1y * b1.y));
		// return T.R.mulT(v.sub(T.position));
	}
	
	public final static void mulTransToOut(final Transform T, final Vec v, final Vec out) {
		final float v1x = v.x - T.position.x;
		final float v1y = v.y - T.position.y;
		final Vec b = T.R.col1;
		final Vec b1 = T.R.col2;
		final float tempy = v1x * b1.x + v1y * b1.y;
		out.x = v1x * b.x + v1y * b.y;
		out.y = tempy;
	}
	
	@Override
	public final String toString() {
		String s = "XForm:\n";
		s += "Position: " + position + "\n";
		s += "R: \n" + R + "\n";
		return s;
	}
}
