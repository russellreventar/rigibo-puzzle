package physics.tools;

/**
 * Global tuning constants based on MKS units and various integer maximums
 * (vertices per shape, pairs, etc.).
 */
public class Settings {
	
	/** A "close to zero" float epsilon value for use */
	public static final float EPSILON = 1.1920928955078125E-7f;
	
	/** Pi. */
	public static final float PI = (float) Math.PI;
	
	// JBox2D specific settings
	/**
	 * needs to be final, or will slow down math methods
	 */
	public static final boolean FAST_MATH = true;
	public static final int CONTACT_STACK_INIT_SIZE = 10;
	public static final boolean SINCOS_LUT_ENABLED = true;
		public static final float SINCOS_LUT_PRECISION = .00011f;
	public static final int SINCOS_LUT_LENGTH = (int) Math.ceil(Math.PI * 2 / SINCOS_LUT_PRECISION);
	/**
	 * Use if the table's precision is large (eg .006 or greater). Although it
	 * is more expensive, it greatly increases accuracy. Look in the MathUtils
	 * source for some test results on the accuracy and speed of lerp vs non
	 * lerp. Or, run the tests yourself in {@link SinCosTest}.
	 */
	public static final boolean SINCOS_LUT_LERP = false;
	
	// Collision
	
	/**
	 * The maximum number of contact points between two convex shapes.
	 */
	public static int maxManifoldPoints = 2;
	
	/**
	 * The maximum number of vertices on a convex polygon.
	 */
	public static int maxPolygonVertices = 8;
	
	/**
	 * This is used to fatten AABBs in the dynamic tree. This allows proxies to
	 * move by a small amount without triggering a tree adjustment. This is in
	 * meters.
	 */
	public static float aabbExtension = 0.1f;
	
	/**
	 * This is used to fatten AABBs in the dynamic tree. This is used to predict
	 * the future position based on the current displacement.
	 * This is a dimensionless multiplier.
	 */
	public static float aabbMultiplier = 2.0f;
	
	/**
	 * A small length used as a collision and constraint tolerance. Usually it
	 * is chosen to be numerically significant, but visually insignificant.
	 */
	public static float linearSlop = 0.005f;
	
	/**
	 * A small angle used as a collision and constraint tolerance. Usually it is
	 * chosen to be numerically significant, but visually insignificant.
	 */
	public static float angularSlop = (2.0f / 180.0f * PI);
	
	/**
	 * The radius of the polygon/edge shape skin. This should not be modified.
	 * Making this smaller means polygons will have and insufficient for
	 * continuous collision. Making it larger may create artifacts for vertex
	 * collision.
	 */
	public static float polygonRadius = (2.0f * linearSlop);
	
	// Dynamics
	
	/**
	 * Maximum number of contacts to be handled to solve a TOI island.
	 */
	public static int maxTOIContacts = 32;
	
	/**
	 * A velocity threshold for elastic collisions. Any collision with a
	 * relative linear velocity below this threshold will be treated as
	 * inelastic.
	 */
	public static float velocityThreshold = 1.0f;
	
	/**
	 * The maximum linear position correction used when solving constraints.
	 * This helps to prevent overshoot.
	 */
	public static float maxLinearCorrection = 0.2f;
	
	/**
	 * The maximum angular position correction used when solving constraints.
	 * This helps to prevent overshoot.
	 */
	public static float maxAngularCorrection = (8.0f / 180.0f * PI);
	
	/**
	 * The maximum linear velocity of a body. This limit is very large and is
	 * used to prevent numerical problems. You shouldn't need to adjust this.
	 */
	public static float maxTranslation = 2.0f;
	public static float maxTranslationSquared = (maxTranslation * maxTranslation);
	
	/**
	 * The maximum angular velocity of a body. This limit is very large and is
	 * used to prevent numerical problems. You shouldn't need to adjust this.
	 */
	public static float maxRotation = (0.5f * PI);
	public static float maxRotationSquared = (maxRotation * maxRotation);
	
	/**
	 * This scale factor controls how fast overlap is resolved. Ideally this
	 * would be 1 so that overlap is removed in one time step. However using
	 * values close to 1 often lead to overshoot.
	 */
	public static float contactBaumgarte = 0.2f;
	
	// Sleep
	
	/**
	 * The time that a body must be still before it will go to sleep.
	 */
	public static float timeToSleep = 0.5f;
	
	/**
	 * A body cannot sleep if its linear velocity is above this tolerance.
	 */
	public static float linearSleepTolerance = 0.01f;
	
	/**
	 * A body cannot sleep if its angular velocity is above this tolerance.
	 */
	public static float angularSleepTolerance = (2.0f / 180.0f * PI);
	
	/**
	 * Friction mixing law. Feel free to customize this.
	 * TODO djm: add customization
	 * 
	 * @param friction1
	 * @param friction2
	 * @return
	 */
	public static final float mixFriction(float friction1, float friction2) {
		return MathUtils.sqrt(friction1 * friction2);
	}
	
	/**
	 * Restitution mixing law. Feel free to customize this.
	 * TODO djm: add customization
	 * 
	 * @param restitution1
	 * @param restitution2
	 * @return
	 */
	public static final float mixRestitution(float restitution1, float restitution2) {
		return restitution1 > restitution2 ? restitution1 : restitution2;
	}
}
