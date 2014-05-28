package physics.collision;


import physics.dynamics.contacts.Contact;

/**
 *
 */
public interface ContactListener {

	/**
	 * Called when two fixtures begin to touch.
	 */
	public void beginContact(Contact contact);
	
	/**
	 * Called when two fixtures cease to touch.
	 */
	public void endContact(Contact contact);
	
	/**
	 * Called after a contact is updated. 
	 * @param contact
	 * @param oldManifold
	 */
	public void preSolve(Contact contact, Manifold oldManifold);
}
