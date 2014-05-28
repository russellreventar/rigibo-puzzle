package physics.collision;

import physics.tools.Vec;

/** An axis-aligned bounding box. */
public class AABB {
	/** Bottom left vertex of bounding box. */
	public final Vec lowerBound;
	/** Top right vertex of bounding box. */
	public final Vec upperBound;

	/**
	 * Creates the default object, with vertices at 0,0 and 0,0.
	 */
	public AABB() {
		lowerBound = new Vec();
		upperBound = new Vec();
	}

	/**
	 * Copies from the given object
	 * 
	 * @param copy
	 *            the object to copy from
	 */
	public AABB(final AABB copy) {
		this(copy.lowerBound, copy.upperBound);
	}

	/**
	 * Creates an AABB object using the given bounding vertices.
	 * 
	 * @param lowerVertex
	 *            the bottom left vertex of the bounding box
	 * @param maxVertex
	 *            the top right vertex of the bounding box
	 */
	public AABB(final Vec lowerVertex, final Vec upperVertex) {
		this.lowerBound = lowerVertex.clone(); // clone to be safe
		this.upperBound = upperVertex.clone();
	}

	/**
	 * Sets this object from the given object
	 * 
	 * @param aabb
	 *            the object to copy from
	 */
	public final void set(final AABB aabb) {
		lowerBound.set(aabb.lowerBound);
		upperBound.set(aabb.upperBound);
	}

	/** Verify that the bounds are sorted */
	public final boolean isValid() {
		final float dx = upperBound.x - lowerBound.x;
		if (dx < 0f) {
			return false;
		}
		final float dy = upperBound.y - lowerBound.y;
		if (dy < 0) {
			return false;
		}
		return lowerBound.isValid() && upperBound.isValid();
	}

	/**
	 * Get the center of the AABB
	 * 
	 * @return
	 */
	public final Vec getCenter() {
		final Vec center = new Vec(lowerBound);
		center.addLocal(upperBound);
		center.mulLocal(.5f);
		return center;
	}

	public final void getCenterToOut(final Vec out) {
		out.x = (lowerBound.x + upperBound.x) * .5f;
		out.y = (lowerBound.y + upperBound.y) * .5f;
	}

	/**
	 * Get the extents of the AABB (half-widths).
	 * 
	 * @return
	 */
	public final Vec getExtents() {
		final Vec center = new Vec(upperBound);
		center.subLocal(lowerBound);
		center.mulLocal(.5f);
		return center;
	}

	public final void getExtentsToOut(final Vec out) {
		out.x = (upperBound.x - lowerBound.x) * .5f;
		out.y = (upperBound.y - lowerBound.y) * .5f; // thanks FDN1
	}

	public final void getVertices(Vec[] argRay) {
		argRay[0].set(lowerBound);
		argRay[1].set(lowerBound);
		argRay[1].x += upperBound.x - lowerBound.x;
		argRay[2].set(upperBound);
		argRay[3].set(upperBound);
		argRay[3].x -= upperBound.x - lowerBound.x;
	}

	/**
	 * Gets the perimeter length
	 * 
	 * @return
	 */
	public final float getPerimeter() {
		return 2.0f * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y);
	}

	/**
	 * Combine two AABBs into this one.
	 * 
	 * @param aabb1
	 * @param aab
	 */
	public final void combine(final AABB aabb1, final AABB aab) {
		lowerBound.x = aabb1.lowerBound.x < aab.lowerBound.x ? aabb1.lowerBound.x
				: aab.lowerBound.x;
		lowerBound.y = aabb1.lowerBound.y < aab.lowerBound.y ? aabb1.lowerBound.y
				: aab.lowerBound.y;
		upperBound.x = aabb1.upperBound.x > aab.upperBound.x ? aabb1.upperBound.x
				: aab.upperBound.x;
		upperBound.y = aabb1.upperBound.y > aab.upperBound.y ? aabb1.upperBound.y
				: aab.upperBound.y;
	}

	/**
	 * Does this aabb contain the provided AABB.
	 * 
	 * @return
	 */
	public final boolean contains(final AABB aabb) {

		return lowerBound.x > aabb.lowerBound.x
				&& lowerBound.y > aabb.lowerBound.y
				&& aabb.upperBound.x > upperBound.x
				&& aabb.upperBound.y > upperBound.y;
	}

	public static final boolean testOverlap(final AABB a, final AABB b) {
		if (b.lowerBound.x - a.upperBound.x > 0.0f
				|| b.lowerBound.y - a.upperBound.y > 0.0f) {
			return false;
		}

		if (a.lowerBound.x - b.upperBound.x > 0.0f
				|| a.lowerBound.y - b.upperBound.y > 0.0f) {
			return false;
		}

		return true;
	}

	@Override
	public final String toString() {
		final String s = "AABB[" + lowerBound + " . " + upperBound + "]";
		return s;
	}
}
