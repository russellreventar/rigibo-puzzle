package physics.dynamics;

import physics.collision.ContactListener;
import physics.collision.PairCallback;
import physics.collision.broadphase.BroadPhase;
import physics.collision.broadphase.DynamicTreeNode;
import physics.dynamics.contacts.Contact;
import physics.dynamics.contacts.ContactEdge;

/**
 * Delegate of World.
 * 
 */
public class ContactManager implements PairCallback {
	
	public BroadPhase m_broadPhase;
	public Contact m_contactList;
	public int m_contactCount;
	public ContactListener m_contactListener;
	
	private final Pool pool;
	
	public ContactManager(Pool argPool) {
		m_contactList = null;
		m_contactCount = 0;
		m_contactListener = null;
		m_broadPhase = new BroadPhase();
		pool = argPool;
	}
	
	/**
	 * Broad-phase callback.
	 * 
	 * @param proxyUserDataA
	 * @param proxyUserDataB
	 */
	public void addPair(Object proxyUserDataA, Object proxyUserDataB) {
		Fixture fixtureA = (Fixture) proxyUserDataA;
		Fixture fixtureB = (Fixture) proxyUserDataB;
		
		RigidBody bodyA = fixtureA.getBody();
		RigidBody bodyB = fixtureB.getBody();
		
		// Are the fixtures on the same body?
		if (bodyA == bodyB) {
			return;
		}
		
		// Does a contact already exist?
		ContactEdge edge = bodyB.getContactList();
		while (edge != null) {
			if (edge.other == bodyA) {
				Fixture fA = edge.contact.getFixtureA();
				Fixture fB = edge.contact.getFixtureB();
				if (fA == fixtureA && fB == fixtureB) {
					// A contact already exists.
					return;
				}
				
				if (fA == fixtureB && fB == fixtureA) {
					// A contact already exists.
					return;
				}
			}
			
			edge = edge.next;
		}
		
		// Does a joint override collision? is at least one body dynamic?
		if (bodyB.shouldCollide(bodyA) == false) {
			return;
		}
		
		// Call the factory.
		Contact c = pool.popContact(fixtureA, fixtureB);
		
		// Contact creation may swap fixtures.
		fixtureA = c.getFixtureA();
		fixtureB = c.getFixtureB();
		bodyA = fixtureA.getBody();
		bodyB = fixtureB.getBody();
		
		// Insert into the world.
		c.m_prev = null;
		c.m_next = m_contactList;
		if (m_contactList != null) {
			m_contactList.m_prev = c;
		}
		m_contactList = c;
		
		// Connect to island graph.
		
		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;
		
		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null) {
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;
		
		// Connect to body B
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;
		
		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null) {
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;
		
		++m_contactCount;
	}
	
	public void findNewContacts() {
		m_broadPhase.updatePairs(this);
	}
	
	public void destroy(Contact c) {
		Fixture fixtureA = c.getFixtureA();
		Fixture fixtureB = c.getFixtureB();
		RigidBody bodyA = fixtureA.getBody();
		RigidBody bodyB = fixtureB.getBody();
		
		if (m_contactListener != null && c.isTouching()) {
			m_contactListener.endContact(c);
		}
		
		// Remove from the world.
		if (c.m_prev != null) {
			c.m_prev.m_next = c.m_next;
		}
		
		if (c.m_next != null) {
			c.m_next.m_prev = c.m_prev;
		}
		
		if (c == m_contactList) {
			m_contactList = c.m_next;
		}
		
		// Remove from body 1
		if (c.m_nodeA.prev != null) {
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}
		
		if (c.m_nodeA.next != null) {
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}
		
		if (c.m_nodeA == bodyA.m_contactList) {
			bodyA.m_contactList = c.m_nodeA.next;
		}
		
		// Remove from body 2
		if (c.m_nodeB.prev != null) {
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}
		
		if (c.m_nodeB.next != null) {
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}
		
		if (c.m_nodeB == bodyB.m_contactList) {
			bodyB.m_contactList = c.m_nodeB.next;
		}
		
		// Call the factory.
		pool.pushContact(c);
		--m_contactCount;
	}
	
	/**
	 * This is the top level collision call for the time step. Here
	 * all the narrow phase collision is processed for the world
	 * contact list.
	 */
	public void collide() {
		// Update awake contacts.
		Contact c = m_contactList;
		while (c != null) {
			Fixture fixtureA = c.getFixtureA();
			Fixture fixtureB = c.getFixtureB();
			RigidBody bodyA = fixtureA.getBody();
			RigidBody bodyB = fixtureB.getBody();
			
			if (bodyA.isAwake() == false && bodyB.isAwake() == false) {
				c = c.getNext();
				continue;
			}
			
			// is this contact flagged for filtering?
			if ((c.m_flags & Contact.FILTER_FLAG) == Contact.FILTER_FLAG) {
				// Should these bodies collide?
				if (bodyB.shouldCollide(bodyA) == false) {
					Contact cNuke = c;
					c = cNuke.getNext();
					destroy(cNuke);
					continue;
				}
								
				// Clear the filtering flag.
				c.m_flags &= ~Contact.FILTER_FLAG;
			}
			
			DynamicTreeNode proxyIdA = fixtureA.m_proxy;
			DynamicTreeNode proxyIdB = fixtureB.m_proxy;
			boolean overlap = m_broadPhase.testOverlap(proxyIdA, proxyIdB);
			
			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (overlap == false) {
				Contact cNuke = c;
				c = cNuke.getNext();
				destroy(cNuke);
				continue;
			}
			
			// The contact persists.
			c.update(m_contactListener);
			c = c.getNext();
		}
	}
}
