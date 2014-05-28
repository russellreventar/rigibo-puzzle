package physics.collision;

/** Contact ids to facilitate warm starting.*/
public class ContactID {

	/** The features that intersect to form the contact point */
	public final Features features;

	/** The features that intersect to form the contact point */
	public static class Features {
		/** The edge that defines the outward contact normal. */
		public int referenceEdge;
		/** The edge most anti-parallel to the reference edge. */
		public int incidentEdge;
		/** The vertex (0 or 1) on the incident edge that was clipped. */
		public int incidentVertex;
		/** A value of 1 indicates that the reference edge is on shape2. */
		public int flip;

		public Features() {
			referenceEdge = incidentEdge = incidentVertex = flip = 0;
		}

		private Features(final Features f) {
			referenceEdge = f.referenceEdge;
			incidentEdge = f.incidentEdge;
			incidentVertex = f.incidentVertex;
			flip = f.flip;
		}

		private void set(final Features f){
			referenceEdge = f.referenceEdge;
			incidentEdge = f.incidentEdge;
			incidentVertex = f.incidentVertex;
			flip = f.flip;
		}

		private boolean isEqual(final Features f){
			return (referenceEdge==f.referenceEdge &&
					incidentEdge==f.incidentEdge &&
					incidentVertex==f.incidentVertex &&
					flip==f.flip);
		}

		@Override
		public String toString() {
			final String s = "Features: (" + this.flip + " ," + this.incidentEdge + " ," + this.incidentVertex + " ," + this.referenceEdge + ")";
			return s;
		}

	}

	public boolean isEqual(final ContactID cid) {
		return cid.features.isEqual(this.features);
	}

	public ContactID() {
		features = new Features();
	}

	public ContactID(final ContactID c) {
		features = new Features(c.features);
	}

	public void set(final ContactID c){
		features.set(c.features);
	}
	
	/**
	 * zeros out the data
	 */
	public void zero() {
		features.flip = 0;
		features.incidentEdge = 0;
		features.incidentVertex = 0;
		features.referenceEdge = 0;
	}

}
