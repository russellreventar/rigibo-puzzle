package physics.dynamics;

import physics.collision.shapes.Shape;

/**
 * A fixture definition is used to create a fixture. This class defines an
 * abstract fixture definition. You can reuse fixture definitions safely.
 *
 */
public class FixtureDef {
	/**
	 * The shape, this must be set. The shape will be cloned, so you
	 * can create the shape on the stack.
	 */
	public Shape shape = null;
	
	/**
	 * Use this to store application specific fixture data.
	 */
	public Object userData;
	
	/**
	 * The friction coefficient, usually in the range [0,1].
	 */
	public float friction;
	
	/**
	 * The restitution (elasticity) usually in the range [0,1].
	 */
	public float restitution;
	
	/**
	 * The density, usually in kg/m^2
	 */
	public float density;
	
	/**
	 * A sensor shape collects contact information but never generates a collision
	 * response.
	 */
	public boolean isSensor;
	
	
	public FixtureDef(){
		shape = null;
		userData = null;
		friction = 0.2f;
		restitution = 0f;
		density = 0f;
		isSensor = false;
	}
}
