package physics.dynamics;

import physics.collision.ContactListener;
import physics.collision.TimeOfImpact.TOIInput;
import physics.collision.TimeOfImpact.TOIOutput;
import physics.collision.TimeOfImpact.TOIOutputState;

import physics.collision.shapes.ShapeType;


import physics.dataStorage.DefaultWorldPool;
import physics.dataStorage.IDynamicStack;
import physics.dataStorage.IWorldPool;
import physics.dynamics.contacts.Contact;
import physics.dynamics.contacts.ContactEdge;
import physics.dynamics.contacts.ContactRegister;
import physics.dynamics.contacts.TOISolver;
import physics.tools.Settings;
import physics.tools.Sweep;
import physics.tools.Vec;


/**
 * The world class manages all physics entities, dynamic simulation,
 * and asynchronous queries. The world also contains efficient memory
 * management facilities.
 * 
 */
public class Pool {
	public static final int WORLD_POOL_SIZE = 100;
	public static final int WORLD_POOL_CONTAINER_SIZE = 10;
	
	public static final int NEW_FIXTURE = 0x0001;
	public static final int LOCKED = 0x0002;
	public static final int CLEAR_FORCES = 0x0004;

	// statistics gathering
	public int activeContacts = 0;
	public int contactPoolCount = 0;
	
	protected int m_flags;
	
	protected ContactManager m_contactManager;
	
	private RigidBody m_bodyList;
	
	private int m_bodyCount;
	private int m_jointCount;
	
	private final Vec m_gravity = new Vec();
	private boolean m_allowSleep;
	
	
	private final IWorldPool pool;
	
	/**
	 * This is used to compute the time step ratio to
	 * support a variable time step.
	 */
	private float m_inv_dt0;
	
	/**
	 * This is for debugging the solver.
	 */
	private boolean m_warmStarting;
	
	/**
	 * This is for debugging the solver.
	 */
	private boolean m_continuousPhysics;
	
	private ContactRegister[][] contactStacks = new ContactRegister[ShapeType.TYPE_COUNT][ShapeType.TYPE_COUNT];
	
	public Pool(Vec gravity, boolean doSleep){
		this(gravity, doSleep,
				new DefaultWorldPool(WORLD_POOL_SIZE, WORLD_POOL_CONTAINER_SIZE));
	}
	
	/**
	 * Construct a world object.
	 * 
	 * @param gravity
	 *            the world gravity vector.
	 * @param doSleep
	 *            improve performance by not simulating inactive bodies.
	 */
	public Pool(Vec gravity, boolean doSleep, IWorldPool argPool) {
		pool = argPool;
		
		m_bodyList = null;
		
		m_bodyCount = 0;
		m_jointCount = 0;
		
		m_warmStarting = true;
		m_continuousPhysics = true;
		
		m_allowSleep = doSleep;
		m_gravity.set(gravity);
		
		m_flags = CLEAR_FORCES;
		
		m_inv_dt0 = 0f;
		
		m_contactManager = new ContactManager(this);
		
		initializeRegisters();
	}
	public void setContactListener(ContactListener listener) {
		m_contactManager.m_contactListener = listener;
	}
	
	public void setAllowSleep(boolean argAllowSleep){
		m_allowSleep = argAllowSleep;
	}
	
	public boolean isAllowSleep(){
		return m_allowSleep;
	}
	
	private void addType(IDynamicStack<Contact> creator, ShapeType type1,
			ShapeType type2) {
		ContactRegister register = new ContactRegister();
		register.creator = creator;
		register.primary = true;
		contactStacks[type1.intValue][type2.intValue] = register;

		if (type1 != type2) {
			ContactRegister register2 = new ContactRegister();
			register2.creator = creator;
			register2.primary = false;
			contactStacks[type2.intValue][type1.intValue] = register2;
		}
	}

	private void initializeRegisters() {
		addType(pool.getCircleContactStack(), ShapeType.CIRCLE, ShapeType.CIRCLE);
		addType(pool.getPolyCircleContactStack(), ShapeType.POLYGON, ShapeType.CIRCLE);
		addType(pool.getPolyContactStack(), ShapeType.POLYGON, ShapeType.POLYGON);
	}

	public Contact popContact(Fixture fixtureA, Fixture fixtureB) {
		final ShapeType type1 = fixtureA.getType();
		final ShapeType type2 = fixtureB.getType();

		final ContactRegister reg = contactStacks[type1.intValue][type2.intValue];
		final IDynamicStack<Contact> creator = reg.creator;
		if (creator != null) {
			if (reg.primary) {
				Contact c = creator.pop();
				c.init(fixtureA, fixtureB);
				return c;
			} else {
				Contact c = creator.pop();
				c.init(fixtureB, fixtureA);
				return c;
			}
		} else {
			return null;
		}
	}

	public void pushContact(Contact contact) {

		if (contact.m_manifold.pointCount > 0) {
			contact.getFixtureA().getBody().setAwake(true);
			contact.getFixtureB().getBody().setAwake(true);
		}

		ShapeType type1 = contact.getFixtureA().getType();
		ShapeType type2 = contact.getFixtureB().getType();

		IDynamicStack<Contact> creator = contactStacks[type1.intValue][type2.intValue].creator;
		creator.push(contact);
	}
	
	public IWorldPool getPool() {
		return pool;
	}
	
	/**
	 * create a rigid body given a definition. No reference to the definition
	 * is retained.
	 * 
	 * @warning This function is locked during callbacks.
	 * @param def
	 * @return
	 */
	public RigidBody createBody(RigidBodyInfo def) {
		assert (isLocked() == false);
		if (isLocked()) {
			return null;
		}
		RigidBody b = new RigidBody(def, this);
		
		// add to world doubly linked list
		b.m_prev = null;
		b.m_next = m_bodyList;
		if (m_bodyList != null) {
			m_bodyList.m_prev = b;
		}
		m_bodyList = b;
		++m_bodyCount;
		
		return b;
	}
	
	/**
	 * destroy a rigid body given a definition. No reference to the definition
	 * is retained. This function is locked during callbacks.
	 * 
	 * @warning This automatically deletes all associated shapes and joints.
	 * @warning This function is locked during callbacks.
	 * @param body
	 */
	public void destroyBody(RigidBody body) {
		assert (m_bodyCount > 0);
		assert (isLocked() == false);
		if (isLocked()) {
			return;
		}
		
		// Delete the attached contacts.
		ContactEdge ce = body.m_contactList;
		while (ce != null) {
			ContactEdge ce0 = ce;
			ce = ce.next;
			m_contactManager.destroy(ce0.contact);
		}
		body.m_contactList = null;
		
		Fixture f = body.m_fixtureList;
		while (f != null) {
			Fixture f0 = f;
			f = f.m_next;

			f0.destroyProxy(m_contactManager.m_broadPhase);
			f0.destroy();
		}
		body.m_fixtureList = null;
		body.m_fixtureCount = 0;
		
		// Remove world body list.
		if (body.m_prev != null) {
			body.m_prev.m_next = body.m_next;
		}
		
		if (body.m_next != null) {
			body.m_next.m_prev = body.m_prev;
		}
		
		if (body == m_bodyList) {
			m_bodyList = body.m_next;
		}
		
		--m_bodyCount;
	}
	
	
	
	// djm pooling
	private final TimeStep step = new TimeStep();
	
	/**
	 * Take a time step. This performs collision detection, integration,
	 * and constraint solution.
	 * 
	 * @param timeStep
	 *            the amount of time to simulate, this should not vary.
	 * @param velocityIterations
	 *            for the velocity constraint solver.
	 * @param positionIterations
	 *            for the position constraint solver.
	 */
	public void step(float dt, int velocityIterations, int positionIterations) {
		// log.debug("Starting step");
		// If new fixtures were added, we need to find the new contacts.
		if ((m_flags & NEW_FIXTURE) == NEW_FIXTURE) {
			m_contactManager.findNewContacts();
			m_flags &= ~NEW_FIXTURE;
		}
		
		m_flags |= LOCKED;
		
		step.dt = dt;
		step.velocityIterations = velocityIterations;
		step.positionIterations = positionIterations;
		if (dt > 0.0f) {
			step.inv_dt = 1.0f / dt;
		}
		else {
			step.inv_dt = 0.0f;
		}
		
		step.dtRatio = m_inv_dt0 * dt;
		
		step.warmStarting = m_warmStarting;
		
		// Update contacts. This is where some contacts are destroyed.
		m_contactManager.collide();
		
		// Integrate velocities, solve velocity constraints, and integrate positions.
		if (step.dt > 0.0f) {
			solve(step);
		}
		
		// Handle TOI events.
		if (m_continuousPhysics && step.dt > 0.0f) {
			solveTOI();
		}
		
		if (step.dt > 0.0f) {
			m_inv_dt0 = step.inv_dt;
		}
		
		if ((m_flags & CLEAR_FORCES) == CLEAR_FORCES) {
			clearForces();
		}
		
		m_flags &= ~LOCKED;
	}
	
	/**
	 * Call this after you are done with time steps to clear the forces. You normally
	 * call this after each call to Step, unless you are performing sub-steps. By default,
	 * forces will be automatically cleared, so you don't need to call this function.
	 * 
	 * @see setAutoClearForces
	 */
	public void clearForces() {
		for (RigidBody body = m_bodyList; body != null; body = body.getNext()) {
			body.m_force.setZero();
			body.m_torque = 0.0f;
		}
	}
	
	/**
	 * Is the world locked (in the middle of a time step).
	 * 
	 * @return
	 */
	public boolean isLocked() {
		return (m_flags & LOCKED) == LOCKED;
	}
	

	private final Island island = new Island();
	private RigidBody[] stack = new RigidBody[10];
	
	private void solve(TimeStep step) {
		// Size the island for the worst case.
		island.init(m_bodyCount, m_contactManager.m_contactCount, m_jointCount, m_contactManager.m_contactListener);
		
		// Clear all the island flags.
		for (RigidBody b = m_bodyList; b != null; b = b.m_next) {
			b.m_flags &= ~RigidBody.e_islandFlag;
		}
		for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next) {
			c.m_flags &= ~Contact.ISLAND_FLAG;
		}

		// Build and simulate all awake islands.
		int stackSize = m_bodyCount;
		if (stack.length < stackSize) {
			stack = new RigidBody[stackSize];
		}
		for (RigidBody seed = m_bodyList; seed != null; seed = seed.m_next) {
			if ((seed.m_flags & RigidBody.e_islandFlag) == RigidBody.e_islandFlag) {
				continue;
			}
			
			if (seed.isAwake() == false || seed.isActive() == false) {
				continue;
			}
			
			// The seed can be dynamic or kinematic.
			if (seed.getType() == RigidBodyType.STATIC) {
				continue;
			}
			
			// Reset island and stack.
			island.clear();
			int stackCount = 0;
			stack[stackCount++] = seed;
			seed.m_flags |= RigidBody.e_islandFlag;
			
			// Perform a depth first search (DFS) on the constraint graph.
			while (stackCount > 0) {
				// Grab the next body off the stack and add it to the island.
				RigidBody b = stack[--stackCount];
				assert (b.isActive() == true);
				island.add(b);
				
				// Make sure the body is awake.
				b.setAwake(true);
				
				// To keep islands as small as possible, we don't
				// propagate islands across static bodies.
				if (b.getType() == RigidBodyType.STATIC) {
					continue;
				}
				
				// Search all contacts connected to this body.
				for (ContactEdge ce = b.m_contactList; ce != null; ce = ce.next) {
					Contact contact = ce.contact;
					
					// Has this contact already been added to an island?
					if ((contact.m_flags & Contact.ISLAND_FLAG) == Contact.ISLAND_FLAG) {
						continue;
					}
					
					// Is this contact solid and touching?
					if (contact.isEnabled() == false || contact.isTouching() == false) {
						continue;
					}
					
					// Skip sensors.
					boolean sensorA = contact.m_fixtureA.m_isSensor;
					boolean sensorB = contact.m_fixtureB.m_isSensor;
					if (sensorA || sensorB) {
						continue;
					}
					
					island.add(contact);
					contact.m_flags |= Contact.ISLAND_FLAG;
					
					RigidBody other = ce.other;
					
					// Was the other body already added to this island?
					if ((other.m_flags & RigidBody.e_islandFlag) == RigidBody.e_islandFlag) {
						continue;
					}
					
					assert (stackCount < stackSize);
					stack[stackCount++] = other;
					other.m_flags |= RigidBody.e_islandFlag;
				}
				
			}
			
			island.solve(step, m_gravity, m_allowSleep);
			
			// Post solve cleanup.
			for (int i = 0; i < island.m_bodyCount; ++i) {
				// Allow static bodies to participate in other islands.
				RigidBody b = island.m_bodies[i];
				if (b.getType() == RigidBodyType.STATIC) {
					b.m_flags &= ~RigidBody.e_islandFlag;
				}
			}
		}
		
		// Synchronize fixtures, check for out of range bodies.
		for (RigidBody b = m_bodyList; b != null; b = b.getNext()) {
			// If a body was not in an island then it did not move.
			if ((b.m_flags & RigidBody.e_islandFlag) == 0) {
				continue;
			}
			
			if (b.getType() == RigidBodyType.STATIC) {
				continue;
			}
			
			// Update fixtures (for broad-phase).
			b.synchronizeFixtures();
		}
		
		// Look for new contacts.
		m_contactManager.findNewContacts();
	}
	
	private void solveTOI() {
		// Prepare all contacts.
		for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next) {
			// Enable the contact
			c.m_flags |= Contact.ENABLED_FLAG;
			
			// Set the number of TOI events for this contact to zero.
			c.m_toiCount = 0;
		}
		
		// Initialize the TOI flag.
		for (RigidBody body = m_bodyList; body != null; body = body.m_next) {
			// Kinematic, and static bodies will not be affected by the TOI event.
			// If a body was not in an island then it did not move.
			if ((body.m_flags & RigidBody.e_islandFlag) == 0 || body.getType() == RigidBodyType.KINEMATIC
					|| body.getType() == RigidBodyType.STATIC) {
				body.m_flags |= RigidBody.e_toiFlag;
			}
			else {
				body.m_flags &= ~RigidBody.e_toiFlag;
			}
		}
		
		// Collide non-bullets.
		for (RigidBody body = m_bodyList; body != null; body = body.m_next) {
			if ((body.m_flags & RigidBody.e_toiFlag) == RigidBody.e_toiFlag) {
				continue;
			}
			
			if (body.isBullet() == true) {
				continue;
			}
			
			solveTOI(body);
			
			body.m_flags |= RigidBody.e_toiFlag;
		}
		
		// Collide bullets.
		for (RigidBody body = m_bodyList; body != null; body = body.m_next) {
			if ((body.m_flags & RigidBody.e_toiFlag) == RigidBody.e_toiFlag) {
				continue;
			}
			
			if (body.isBullet() == false) {
				continue;
			}
			
			solveTOI(body);
			
			body.m_flags |= RigidBody.e_toiFlag;
		}
	}
	
	private final TOIInput toiInput = new TOIInput();
	private final TOIOutput toiOutput = new TOIOutput();
	private final Sweep backup = new Sweep();
	private final TOISolver toiSolver = new TOISolver();
	

	private Contact[] m_contacts = new Contact[Settings.maxTOIContacts];
	
	private void solveTOI(RigidBody body) {
		// Find the minimum contact.
		Contact toiContact = null;
		float toi = 1.0f;
		RigidBody toiOther = null;
		boolean found;
		int count;
		int iter = 0;
		
		boolean bullet = body.isBullet();
		
		// Iterate until all contacts agree on the minimum TOI. We have
		// to iterate because the TOI algorithm may skip some intermediate
		// collisions when objects rotate through each other.
		do {
			count = 0;
			found = false;
			for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next) {
				if (ce.contact == toiContact) {
					continue;
				}
				
				RigidBody other = ce.other;
				RigidBodyType type = other.getType();
				
				// Only bullets perform TOI with dynamic bodies.
				if (bullet == true) {
					// Bullets only perform TOI with bodies that have their TOI resolved.
					if ((other.m_flags & RigidBody.e_toiFlag) == 0) {
						continue;
					}
					
					// No repeated hits on non-static bodies
					if (type != RigidBodyType.STATIC && (ce.contact.m_flags & Contact.BULLET_HIT_FLAG) != 0) {
						continue;
					}
				}
				else if (type == RigidBodyType.DYNAMIC) {
					continue;
				}
				
				// Check for a disabled contact.
				Contact contact = ce.contact;
				if (contact.isEnabled() == false) {
					continue;
				}
				
				// Prevent infinite looping.
				if (contact.m_toiCount > 10) {
					continue;
				}
				
				Fixture fixtureA = contact.m_fixtureA;
				Fixture fixtureB = contact.m_fixtureB;
				
				// Cull sensors.
				if (fixtureA.isSensor() || fixtureB.isSensor()) {
					continue;
				}
				
				RigidBody bodyA = fixtureA.m_body;
				RigidBody bodyB = fixtureB.m_body;
				
				// Compute the time of impact in interval [0, minTOI]
				toiInput.proxyA.set(fixtureA.getShape());
				toiInput.proxyB.set(fixtureB.getShape());
				toiInput.sweepA.set(bodyA.m_sweep);
				toiInput.sweepB.set(bodyB.m_sweep);
				toiInput.tMax = toi;
				
				pool.getTimeOfImpact().timeOfImpact(toiOutput, toiInput);
				
				if (toiOutput.state == TOIOutputState.TOUCHING && toiOutput.t < toi) {
					toiContact = contact;
					toi = toiOutput.t;
					toiOther = other;
					found = true;
				}
				
				++count;
			}
			
			++iter;
		}
		while (found && count > 1 && iter < 50);
		
		if (toiContact == null) {
			body.advance(1.0f);
			return;
		}
		
		backup.set(body.m_sweep);
		body.advance(toi);
		toiContact.update(m_contactManager.m_contactListener);
		if (toiContact.isEnabled() == false) {
			// Contact disabled. Backup and recurse.
			body.m_sweep.set(backup);
			solveTOI(body);
		}
		
		++toiContact.m_toiCount;
		
		// Update all the valid contacts on this body and build a contact island.
		if (m_contacts == null || m_contacts.length < Settings.maxTOIContacts){
			m_contacts = new Contact[Settings.maxTOIContacts];
		}
		
		count = 0;
		for (ContactEdge ce = body.m_contactList; ce != null && count < Settings.maxTOIContacts; ce = ce.next) {
			RigidBody other = ce.other;
			RigidBodyType type = other.getType();
			
			// Only perform correction with static bodies, so the
			// body won't get pushed out of the world.
			if (type == RigidBodyType.DYNAMIC) {
				continue;
			}
			
			// Check for a disabled contact.
			Contact contact = ce.contact;
			if (contact.isEnabled() == false) {
				continue;
			}
			
			Fixture fixtureA = contact.m_fixtureA;
			Fixture fixtureB = contact.m_fixtureB;
			
			// Cull sensors.
			if (fixtureA.isSensor() || fixtureB.isSensor()) {
				continue;
			}
			
			// The contact likely has some new contact points. The listener
			// gives the user a chance to disable the contact.
			if (contact != toiContact) {
				contact.update(m_contactManager.m_contactListener);
			}
			
			// Did the user disable the contact?
			if (contact.isEnabled() == false) {
				// Skip this contact.
				continue;
			}
			
			if (contact.isTouching() == false) {
				continue;
			}
			
			m_contacts[count] = contact;
			++count;
		}
		
		// Reduce the TOI body's overlap with the contact island.
		toiSolver.initialize(m_contacts, count, body);
		
		float k_toiBaumgarte = 0.75f;
		// boolean solved = false;
		for (int i = 0; i < 20; ++i) {
			boolean contactsOkay = toiSolver.solve(k_toiBaumgarte);
			if (contactsOkay) {
				// solved = true;
				break;
			}
		}
		
		if (toiOther.getType() != RigidBodyType.STATIC) {
			toiContact.m_flags |= Contact.BULLET_HIT_FLAG;
		}
	}
}
