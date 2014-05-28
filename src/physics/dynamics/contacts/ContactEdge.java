package physics.dynamics.contacts;

import physics.dynamics.RigidBody;

/**
 * A contact edge is used to connect bodies and contacts together
 * in a contact graph where each body is a node and each contact
 * is an edge. A contact edge belongs to a doubly linked list
 * maintained in each attached body. Each contact has two contact
 * nodes, one for each attached body.
 *
 */
public class ContactEdge {
	
	/**
	 * provides quick access to the other body attached.
	 */
	public RigidBody other = null;	
	
	/**
	 * the contact
	 */
	public Contact contact = null;
	
	/**
	 * the previous contact edge in the body's contact list
	 */
	public ContactEdge prev = null;	
	
	/**
	 * the next contact edge in the body's contact list
	 */
	public ContactEdge next = null;
}
