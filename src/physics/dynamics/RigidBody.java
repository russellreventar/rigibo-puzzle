package physics.dynamics;


import physics.collision.broadphase.BroadPhase;
import physics.collision.shapes.Mass;
import physics.collision.shapes.Shape;
import physics.dynamics.contacts.Contact;
import physics.dynamics.contacts.ContactEdge;
import physics.tools.Mat22;
import physics.tools.MathUtils;
import physics.tools.Sweep;
import physics.tools.Transform;
import physics.tools.Vec;

/**
 * A rigid body. These are created via World.createBody.
 * 
 */
public class RigidBody {
	public static final int e_islandFlag = 0x0001;
	public static final int e_awakeFlag = 0x0002;
	public static final int e_autoSleepFlag = 0x0004;
	public static final int e_bulletFlag = 0x0008;
	public static final int e_fixedRotationFlag = 0x0010;
	public static final int e_activeFlag = 0x0020;
	public static final int e_toiFlag = 0x0040;

	public RigidBodyType m_type;

	public int m_flags;

	public int m_islandIndex;

	/**
	 * The body origin transform.
	 */
	public final Transform m_xf = new Transform();

	/**
	 * The swept motion for CCD
	 */
	public final Sweep m_sweep = new Sweep();

	public final Vec m_linearVelocity = new Vec();
	public float m_angularVelocity = 0;

	public final Vec m_force = new Vec();
	public float m_torque = 0;

	public Pool m_world;
	public RigidBody m_prev;
	public RigidBody m_next;

	public Fixture m_fixtureList;
	public int m_fixtureCount;

	public ContactEdge m_contactList;

	public float m_mass, m_invMass;

	// Rotational inertia about the center of mass.
	public float m_I, m_invI;

	public float m_linearDamping;
	public float m_angularDamping;

	public float m_sleepTime;

	public Object m_userData;

	public RigidBody(final RigidBodyInfo bd, Pool world) {
		assert (bd.position.isValid());
		assert (bd.linearVelocity.isValid());
		assert (bd.inertiaScale >= 0.0f);
		assert (bd.angularDamping >= 0.0f);
		assert (bd.linearDamping >= 0.0f);

		m_flags = 0;

		if (bd.bullet) {
			m_flags |= e_bulletFlag;
		}
		if (bd.fixedRotation) {
			m_flags |= e_fixedRotationFlag;
		}
		if (bd.allowSleep) {
			m_flags |= e_autoSleepFlag;
		}
		if (bd.awake) {
			m_flags |= e_awakeFlag;
		}
		if (bd.active) {
			m_flags |= e_activeFlag;
		}

		m_world = world;

		m_xf.position.set(bd.position);
		m_xf.R.set(bd.angle);

		m_sweep.localCenter.setZero();
		m_sweep.a0 = m_sweep.a = bd.angle;

		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);

		m_contactList = null;
		m_prev = null;
		m_next = null;

		m_linearVelocity.set(bd.linearVelocity);
		m_angularVelocity = bd.angularVelocity;

		m_linearDamping = bd.linearDamping;
		m_angularDamping = bd.angularDamping;

		m_force.setZero();
		m_torque = 0.0f;

		m_sleepTime = 0.0f;

		m_type = bd.type;

		if (m_type == RigidBodyType.DYNAMIC) {
			m_mass = 1f;
			m_invMass = 1f;
		} else {
			m_mass = 0f;
			m_invMass = 0f;
		}

		m_I = 0.0f;
		m_invI = 0.0f;

		m_userData = bd.userData;

		m_fixtureList = null;
		m_fixtureCount = 0;
	}

	// TODO djm: check out about this new fixture here
	/**
	 * Creates a fixture and attach it to this body. Use this function if you
	 * need to set some fixture parameters, like friction. Otherwise you can
	 * create the fixture directly from a shape. If the density is non-zero,
	 * this function automatically updates the mass of the body. Contacts are
	 * not created until the next time step.
	 * 
	 * @param def
	 *            the fixture definition.
	 * @warning This function is locked during callbacks.
	 */
	public final Fixture createFixture(FixtureDef def) {
		assert (m_world.isLocked() == false);

		if (m_world.isLocked() == true) {
			return null;
		}

		// djm TODO from pool?
		Fixture fixture = new Fixture();
		fixture.create(this, def);

		if ((m_flags & e_activeFlag) == e_activeFlag) {
			BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			fixture.createProxy(broadPhase, m_xf);
		}

		fixture.m_next = m_fixtureList;
		m_fixtureList = fixture;
		++m_fixtureCount;

		fixture.m_body = this;

		// Adjust mass properties if needed.
		if (fixture.m_density > 0.0f) {
			resetMassData();
		}

		// Let the world know we have a new fixture. This will cause new
		// contacts
		// to be created at the beginning of the next time step.
		m_world.m_flags |= Pool.NEW_FIXTURE;

		return fixture;
	}

	private final FixtureDef fixDef = new FixtureDef();

	/**
	 * Creates a fixture from a shape and attach it to this body. This is a
	 * convenience function. Use FixtureDef if you need to set parameters like
	 * friction, restitution, user data, or filtering. If the density is
	 * non-zero, this function automatically updates the mass of the body.
	 * 
	 * @param shape
	 *            the shape to be cloned.
	 * @param density
	 *            the shape density (set to zero for static bodies).
	 * @warning This function is locked during callbacks.
	 */
	public final Fixture createFixture(Shape shape, float density) {
		fixDef.shape = shape;
		fixDef.density = density;

		return createFixture(fixDef);
	}

	/**
	 * Destroy a fixture. This removes the fixture from the broad-phase and
	 * destroys all contacts associated with this fixture. This will
	 * automatically adjust the mass of the body if the body is dynamic and the
	 * fixture has positive density. All fixtures attached to a body are
	 * implicitly destroyed when the body is destroyed.
	 * 
	 * @param fixture
	 *            the fixture to be removed.
	 * @warning This function is locked during callbacks.
	 */
	public final void destroyFixture(Fixture fixture) {
		assert (m_world.isLocked() == false);
		if (m_world.isLocked() == true) {
			return;
		}

		assert (fixture.m_body == this);

		// Remove the fixture from this body's singly linked list.
		assert (m_fixtureCount > 0);
		Fixture node = m_fixtureList;
		Fixture last = null; // java change
		boolean found = false;
		while (node != null) {
			if (node == fixture) {
				node = fixture.m_next;
				found = true;
				break;
			}
			last = node;
			node = node.m_next;
		}

		// You tried to remove a shape that is not attached to this body.
		assert (found);

		// java change, remove it from the list
		if (last == null) {
			m_fixtureList = fixture.m_next;
		} else {
			last.m_next = fixture.m_next;
		}

		// Destroy any contacts associated with the fixture.
		ContactEdge edge = m_contactList;
		while (edge != null) {
			Contact c = edge.contact;
			edge = edge.next;

			Fixture fixtureA = c.getFixtureA();
			Fixture fixtureB = c.getFixtureB();

			if (fixture == fixtureA || fixture == fixtureB) {
				// This destroys the contact and removes it from
				// this body's contact list.
				m_world.m_contactManager.destroy(c);
			}
		}

		if ((m_flags & e_activeFlag) == e_activeFlag) {
			assert (fixture.m_proxy != null);
			BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			fixture.destroyProxy(broadPhase);
		} else {
			assert (fixture.m_proxy == null);
		}

		fixture.destroy();
		fixture.m_body = null;
		fixture.m_next = null;
		fixture = null;

		--m_fixtureCount;

		// Reset the mass data.
		resetMassData();
	}

	/**
	 * Set the position of the body's origin and rotation. This breaks any
	 * contacts and wakes the other bodies. Manipulating a body's transform may
	 * cause non-physical behavior.
	 * 
	 * @param position
	 *            the world position of the body's local origin.
	 * @param angle
	 *            the world rotation in radians.
	 */
	public final void setTransform(Vec position, float angle) {
		assert (m_world.isLocked() == false);
		if (m_world.isLocked() == true) {
			return;
		}

		m_xf.R.set(angle);
		m_xf.position.set(position);

		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);

		m_sweep.a0 = m_sweep.a = angle;

		BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
		for (Fixture f = m_fixtureList; f != null; f = f.m_next) {
			f.synchronize(broadPhase, m_xf, m_xf);
		}

		m_world.m_contactManager.findNewContacts();
	}

	/**
	 * Get the body transform for the body's origin.
	 * 
	 * @return the world transform of the body's origin.
	 */
	public final Transform getTransform() {
		return m_xf;
	}

	/**
	 * Get the world body origin position. Do not modify.
	 * 
	 * @return the world position of the body's origin.
	 */
	public final Vec getPosition() {
		return m_xf.position;
	}

	/**
	 * Get the angle in radians.
	 * 
	 * @return the current world rotation angle in radians.
	 */
	public final float getAngle() {
		return m_sweep.a;
	}

	/**
	 * Get the world position of the center of mass. Do not modify.
	 */
	public final Vec getWorldCenter() {
		return m_sweep.c;
	}

	/**
	 * Get the local position of the center of mass. Do not modify.
	 */
	public final Vec getLocalCenter() {
		return m_sweep.localCenter;
	}

	/**
	 * Set the linear velocity of the center of mass.
	 * 
	 * @param v
	 *            the new linear velocity of the center of mass.
	 */
	public final void setLinearVelocity(Vec v) {
		if (m_type == RigidBodyType.STATIC) {
			return;
		}

		if (Vec.dot(v, v) > 0.0f) {
			setAwake(true);
		}

		m_linearVelocity.set(v);
	}

	/**
	 * Get the linear velocity of the center of mass. Do not modify, instead use
	 * {@link #setLinearVelocity(Vec)}.
	 * 
	 * @return the linear velocity of the center of mass.
	 */
	public final Vec getLinearVelocity() {
		return m_linearVelocity;
	}

	/**
	 * Set the angular velocity.
	 * 
	 * @param omega
	 *            the new angular velocity in radians/second.
	 */
	public final void setAngularVelocity(float w) {
		if (m_type == RigidBodyType.STATIC) {
			return;
		}

		if (w * w > 0f) {
			setAwake(true);
		}

		m_angularVelocity = w;
	}

	/**
	 * Get the angular velocity.
	 * 
	 * @return the angular velocity in radians/second.
	 */
	public final float getAngularVelocity() {
		return m_angularVelocity;
	}

	/**
	 * Apply a force at a world point. If the force is not applied at the center
	 * of mass, it will generate a torque and affect the angular velocity. This
	 * wakes up the body.
	 * 
	 * @param force
	 *            the world force vector, usually in Newtons (N).
	 * @param point
	 *            the world position of the point of application.
	 */
	public final void applyForce(Vec force, Vec point) {
		if (m_type != RigidBodyType.DYNAMIC) {
			return;
		}

		if (isAwake() == false) {
			setAwake(true);
		}

		m_force.x += force.x;
		m_force.y += force.y;

		m_torque += (point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y)
				* force.x;
	}

	/**
	 * Apply a torque. This affects the angular velocity without affecting the
	 * linear velocity of the center of mass. This wakes up the body.
	 * 
	 * @param torque
	 *            about the z-axis (out of the screen), usually in N-m.
	 */
	public final void applyTorque(float torque) {
		if (m_type != RigidBodyType.DYNAMIC) {
			return;
		}

		if (isAwake() == false) {
			setAwake(true);
		}

		m_torque += torque;
	}

	/**
	 * Apply an impulse at a point. This immediately modifies the velocity. It
	 * also modifies the angular velocity if the point of application is not at
	 * the center of mass. This wakes up the body.
	 * 
	 * @param impulse
	 *            the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point
	 *            the world position of the point of application.
	 */
	public final void applyLinearImpulse(Vec impulse, Vec point) {
		if (m_type != RigidBodyType.DYNAMIC) {
			return;
		}

		if (isAwake() == false) {
			setAwake(true);
		}

		// Vec2 temp = tltemp.get();
		// temp.set(impulse).mulLocal(m_invMass);
		// m_linearVelocity.addLocal(temp);
		//
		// temp.set(point).subLocal(m_sweep.c);
		// m_angularVelocity += m_invI * Vec2.cross(temp, impulse);

		m_linearVelocity.x += impulse.x * m_invMass;
		m_linearVelocity.y += impulse.y * m_invMass;

		m_angularVelocity += m_invI
				* ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y)
						* impulse.x);
	}

	/**
	 * Apply an angular impulse.
	 * 
	 * @param impulse
	 *            the angular impulse in units of kg*m*m/s
	 */
	public void applyAngularImpulse(float impulse) {
		if (m_type != RigidBodyType.DYNAMIC) {
			return;
		}

		if (isAwake() == false) {
			setAwake(true);
		}
		m_angularVelocity += m_invI * impulse;
	}

	/**
	 * Get the total mass of the body.
	 * 
	 * @return the mass, usually in kilograms (kg).
	 */
	public final float getMass() {
		return m_mass;
	}

	/**
	 * Get the central rotational inertia of the body.
	 * 
	 * @return the rotational inertia, usually in kg-m^2.
	 */
	public final float getInertia() {
		return m_I
				+ m_mass
				* (m_sweep.localCenter.x * m_sweep.localCenter.x + m_sweep.localCenter.y
						* m_sweep.localCenter.y);
	}

	/**
	 * Get the mass data of the body. The rotational inertia is relative to the
	 * center of mass.
	 * 
	 * @return a struct containing the mass, inertia and center of the body.
	 */
	public final void getMassData(Mass data) {

		data.mass = m_mass;
		data.I = m_I
				+ m_mass
				* (m_sweep.localCenter.x * m_sweep.localCenter.x + m_sweep.localCenter.y
						* m_sweep.localCenter.y);
		data.center.x = m_sweep.localCenter.x;
		data.center.y = m_sweep.localCenter.y;
	}

	/**
	 * Set the mass properties to override the mass properties of the fixtures.
	 * Note that this changes the center of mass position. Note that creating or
	 * destroying fixtures can also alter the mass. This function has no effect
	 * if the body isn't dynamic.
	 * 
	 * @param massData
	 *            the mass properties.
	 */
	public final void setMassData(Mass massData) {
		// TODO_ERIN adjust linear velocity and torque to account for movement
		// of center.
		assert (m_world.isLocked() == false);
		if (m_world.isLocked() == true) {
			return;
		}

		if (m_type != RigidBodyType.DYNAMIC) {
			return;
		}

		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;

		m_mass = massData.mass;
		if (m_mass <= 0.0f) {
			m_mass = 1f;
		}

		m_invMass = 1.0f / m_mass;

		if (massData.I > 0.0f && (m_flags & e_fixedRotationFlag) == 0) {
			m_I = massData.I - m_mass
					* Vec.dot(massData.center, massData.center);
			assert (m_I > 0.0f);
			m_invI = 1.0f / m_I;
		}

		final Vec oldCenter = m_world.getPool().popVec2();
		// Move center of mass.
		oldCenter.set(m_sweep.c);
		m_sweep.localCenter.set(massData.center);

		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);

		// Update center of mass velocity.
		final Vec temp = m_world.getPool().popVec2();
		temp.set(m_sweep.c).subLocal(oldCenter);
		Vec.crossToOut(m_angularVelocity, temp, temp);
		m_linearVelocity.addLocal(temp);

		m_world.getPool().pushVec2(2);
	}

	private final Mass pmd = new Mass();

	/**
	 * This resets the mass properties to the sum of the mass properties of the
	 * fixtures. This normally does not need to be called unless you called
	 * setMassData to override the mass and you later want to reset the mass.
	 */
	public final void resetMassData() {
		// Compute mass data from shapes. Each shape has its own density.
		m_mass = 0.0f;
		m_invMass = 0.0f;
		m_I = 0.0f;
		m_invI = 0.0f;
		m_sweep.localCenter.setZero();

		// Static and kinematic bodies have zero mass.
		if (m_type == RigidBodyType.STATIC || m_type == RigidBodyType.KINEMATIC) {
			// m_sweep.c0 = m_sweep.c = m_xf.position;
			m_sweep.c.set(m_xf.position);
			m_sweep.c0.set(m_xf.position);
			return;
		}

		assert (m_type == RigidBodyType.DYNAMIC);

		// Accumulate mass over all fixtures.
		final Vec center = m_world.getPool().popVec2();
		center.setZero();
		final Vec temp = m_world.getPool().popVec2();
		final Mass massData = pmd;
		for (Fixture f = m_fixtureList; f != null; f = f.m_next) {
			if (f.m_density == 0.0f) {
				continue;
			}
			f.getMassData(massData);
			m_mass += massData.mass;
			// center += massData.mass * massData.center;
			temp.set(massData.center).mulLocal(massData.mass);
			center.addLocal(temp);
			m_I += massData.I;
		}

		// Compute center of mass.
		if (m_mass > 0.0f) {
			m_invMass = 1.0f / m_mass;
			center.mulLocal(m_invMass);
		} else {
			// Force all dynamic bodies to have a positive mass.
			m_mass = 1.0f;
			m_invMass = 1.0f;
		}

		if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0) {
			// Center the inertia about the center of mass.
			m_I -= m_mass * Vec.dot(center, center);
			assert (m_I > 0.0f);
			m_invI = 1.0f / m_I;
		} else {
			m_I = 0.0f;
			m_invI = 0.0f;
		}

		Vec oldCenter = m_world.getPool().popVec2();
		// Move center of mass.
		oldCenter.set(m_sweep.c);
		m_sweep.localCenter.set(center);
		Transform.mulToOut(m_xf, m_sweep.localCenter, m_sweep.c0);
		m_sweep.c.set(m_sweep.c0);

		// Update center of mass velocity.
		temp.set(m_sweep.c).subLocal(oldCenter);
		Vec.crossToOut(m_angularVelocity, temp, temp);
		m_linearVelocity.addLocal(temp);

		m_world.getPool().pushVec2(3);
	}

	/**
	 * Get the world coordinates of a point given the local coordinates.
	 * 
	 * @param localPoint
	 *            a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 */
	public final Vec getWorldPoint(Vec localPoint) {
		Vec v = new Vec();
		getWorldPointToOut(localPoint, v);
		return v;
	}

	public final void getWorldPointToOut(Vec localPoint, Vec out) {
		Transform.mulToOut(m_xf, localPoint, out);
	}

	/**
	 * Get the world coordinates of a vector given the local coordinates.
	 * 
	 * @param localVector
	 *            a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
	public final Vec getWorldVector(Vec localVector) {
		Vec out = new Vec();
		getWorldVectorToOut(localVector, out);
		return out;
	}

	public final void getWorldVectorToOut(Vec localVector, Vec out) {
		Mat22.mulToOut(m_xf.R, localVector, out);
	}

	/**
	 * Gets a local point relative to the body's origin given a world point.
	 * 
	 * @param a
	 *            point in world coordinates.
	 * @return the corresponding local point relative to the body's origin.
	 */
	public final Vec getLocalPoint(Vec worldPoint) {
		Vec out = new Vec();
		getLocalPointToOut(worldPoint, out);
		return out;
	}

	public final void getLocalPointToOut(Vec worldPoint, Vec out) {
		Transform.mulTransToOut(m_xf, worldPoint, out);
	}

	/**
	 * Gets a local vector given a world vector.
	 * 
	 * @param a
	 *            vector in world coordinates.
	 * @return the corresponding local vector.
	 */
	public final Vec getLocalVector(Vec worldVector) {
		Vec out = new Vec();
		getLocalVectorToOut(worldVector, out);
		return out;
	}

	public final void getLocalVectorToOut(Vec worldVector, Vec out) {
		Mat22.mulTransToOut(m_xf.R, worldVector, out);
	}

	/**
	 * Get the world linear velocity of a world point attached to this body.
	 * 
	 * @param a
	 *            point in world coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec getLinearVelocityFromWorldPoint(Vec worldPoint) {
		Vec out = new Vec();
		getLinearVelocityFromWorldPointToOut(worldPoint, out);
		return out;
	}

	public final void getLinearVelocityFromWorldPointToOut(Vec worldPoint,
			Vec out) {
		out.set(worldPoint).subLocal(m_sweep.c);
		Vec.crossToOut(m_angularVelocity, out, out);
		out.addLocal(m_linearVelocity);
	}

	/**
	 * Get the world velocity of a local point.
	 * 
	 * @param a
	 *            point in local coordinates.
	 * @return the world velocity of a point.
	 */
	public final Vec getLinearVelocityFromLocalPoint(Vec localPoint) {
		Vec out = new Vec();
		getLinearVelocityFromLocalPointToOut(localPoint, out);
		return out;
	}

	public final void getLinearVelocityFromLocalPointToOut(Vec localPoint,
			Vec out) {
		getWorldPointToOut(localPoint, out);
		getLinearVelocityFromWorldPointToOut(out, out);
	}

	/** Get the linear damping of the body. */
	public final float getLinearDamping() {
		return m_linearDamping;
	}

	/** Set the linear damping of the body. */
	public final void setLinearDamping(float linearDamping) {
		m_linearDamping = linearDamping;
	}

	/** Get the angular damping of the body. */
	public final float getAngularDamping() {
		return m_angularDamping;
	}

	/** Set the angular damping of the body. */
	public final void setAngularDamping(float angularDamping) {
		m_angularDamping = angularDamping;
	}

	public RigidBodyType getType() {
		return m_type;
	}

	/**
	 * Set the type of this body. This may alter the mass and velocity.
	 * 
	 * @param type
	 */
	public void setType(RigidBodyType type) {
		if (m_type == type) {
			return;
		}

		m_type = type;

		resetMassData();

		if (m_type == RigidBodyType.STATIC) {
			m_linearVelocity.setZero();
			m_angularVelocity = 0.0f;
		}

		setAwake(true);

		m_force.setZero();
		m_torque = 0.0f;

		// Since the body type changed, we need to flag contacts for filtering.
		for (ContactEdge ce = m_contactList; ce != null; ce = ce.next) {
			ce.contact.flagForFiltering();
		}
	}

	/** Is this body treated like a bullet for continuous collision detection? */
	public final boolean isBullet() {
		return (m_flags & e_bulletFlag) == e_bulletFlag;
	}

	/**
	 * Should this body be treated like a bullet for continuous collision
	 * detection?
	 */
	public final void setBullet(boolean flag) {
		if (flag) {
			m_flags |= e_bulletFlag;
		} else {
			m_flags &= ~e_bulletFlag;
		}
	}

	/**
	 * You can disable sleeping on this body. If you disable sleeping, the body
	 * will be woken.
	 * 
	 * @param flag
	 */
	public void setSleepingAllowed(boolean flag) {
		if (flag) {
			m_flags |= e_autoSleepFlag;
		} else {
			m_flags &= ~e_autoSleepFlag;
			setAwake(true);
		}
	}

	/**
	 * Is this body allowed to sleep
	 * 
	 * @return
	 */
	public boolean isSleepingAllowed() {
		return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
	}

	/**
	 * Set the sleep state of the body. A sleeping body has very low CPU cost.
	 * 
	 * @param flag
	 *            set to true to put body to sleep, false to wake it.
	 * @param flag
	 */
	public void setAwake(boolean flag) {
		if (flag) {
			if ((m_flags & e_awakeFlag) == 0) {
				m_flags |= e_awakeFlag;
				m_sleepTime = 0.0f;
			}
		} else {
			m_flags &= ~e_awakeFlag;
			m_sleepTime = 0.0f;
			m_linearVelocity.setZero();
			m_angularVelocity = 0.0f;
			m_force.setZero();
			m_torque = 0.0f;
		}
	}

	/**
	 * Get the sleeping state of this body.
	 * 
	 * @return true if the body is sleeping.
	 */
	public boolean isAwake() {
		return (m_flags & e_awakeFlag) == e_awakeFlag;
	}

	/**
	 * Set the active state of the body. An inactive body is not simulated and
	 * cannot be collided with or woken up. If you pass a flag of true, all
	 * fixtures will be added to the broad-phase. If you pass a flag of false,
	 * all fixtures will be removed from the broad-phase and all contacts will
	 * be destroyed. Fixtures and joints are otherwise unaffected. You may
	 * continue to create/destroy fixtures and joints on inactive bodies.
	 * Fixtures on an inactive body are implicitly inactive and will not
	 * participate in collisions, ray-casts, or queries. Joints connected to an
	 * inactive body are implicitly inactive. An inactive body is still owned by
	 * a World object and remains in the body list.
	 * 
	 * @param flag
	 */
	public void setActive(boolean flag) {
		if (flag == isActive()) {
			return;
		}

		if (flag) {
			m_flags |= e_activeFlag;

			// Create all proxies.
			BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			for (Fixture f = m_fixtureList; f != null; f = f.m_next) {
				f.createProxy(broadPhase, m_xf);
			}

			// Contacts are created the next time step.
		} else {
			m_flags &= ~e_activeFlag;

			// Destroy all proxies.
			BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			for (Fixture f = m_fixtureList; f != null; f = f.m_next) {
				f.destroyProxy(broadPhase);
			}

			// Destroy the attached contacts.
			ContactEdge ce = m_contactList;
			while (ce != null) {
				ContactEdge ce0 = ce;
				ce = ce.next;
				m_world.m_contactManager.destroy(ce0.contact);
			}
			m_contactList = null;
		}
	}

	/**
	 * Get the active state of the body.
	 * 
	 * @return
	 */
	public boolean isActive() {
		return (m_flags & e_activeFlag) == e_activeFlag;
	}

	/**
	 * Set this body to have fixed rotation. This causes the mass to be reset.
	 * 
	 * @param flag
	 */
	public void setFixedRotation(boolean flag) {
		if (flag) {
			m_flags |= e_fixedRotationFlag;
		} else {
			m_flags &= ~e_fixedRotationFlag;
		}

		resetMassData();
	}

	/**
	 * Does this body have fixed rotation?
	 * 
	 * @return
	 */
	public boolean isFixedRotation() {
		return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
	}

	/** Get the list of all fixtures attached to this body. */
	public final Fixture getFixtureList() {
		return m_fixtureList;
	}

	/**
	 * Get the list of all contacts attached to this body.
	 * 
	 * @warning this list changes during the time step and you may miss some
	 *          collisions if you don't use ContactListener.
	 */
	public final ContactEdge getContactList() {
		return m_contactList;
	}

	/** Get the next body in the world's body list. */
	public final RigidBody getNext() {
		return m_next;
	}

	/** Get the user data pointer that was provided in the body definition. */
	public final Object getUserData() {
		return m_userData;
	}

	/**
	 * Set the user data. Use this to store your application specific data.
	 */
	public final void setUserData(Object data) {
		m_userData = data;
	}

	/**
	 * Get the parent world of this body.
	 */
	public final Pool getWorld() {
		return m_world;
	}

	// djm pooling
	private final Transform pxf = new Transform();

	protected final void synchronizeFixtures() {
		final Transform xf1 = pxf;
		final float angle = m_sweep.a0;
		final float c = MathUtils.cos(angle), s = MathUtils.sin(angle);
		xf1.R.col1.x = c;
		xf1.R.col2.x = -s;
		xf1.R.col1.y = s;
		xf1.R.col2.y = c;

		// xf1.position = m_sweep.c0 - Mul(xf1.R, m_sweep.localCenter);
		// before inline
		// Mat22.mulToOut(xf1.R, m_sweep.localCenter, xf1.position);
		// xf1.position.mulLocal(-1).addLocal(m_sweep.c0);
		// after inline
		// R.mulToOut(v,out);
		final Mat22 R = xf1.R;
		final Vec v1 = m_sweep.localCenter;
		final Vec v = m_sweep.c0;
		xf1.position.x = v.x - R.col1.x * v1.x - R.col2.x * v1.y;
		xf1.position.y = v.y - R.col1.y * v1.x - R.col2.y * v1.y;
		// end

		final BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
		for (Fixture f = m_fixtureList; f != null; f = f.m_next) {
			f.synchronize(broadPhase, xf1, m_xf);
		}
	}

	public final void synchronizeTransform() {
		// m_xf.R.set(m_sweep.a);
		//
		// //m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
		// Mat22.mulToOut(m_xf.R, m_sweep.localCenter, m_xf.position);
		// m_xf.position.mulLocal(-1).addLocal(m_sweep.c);
		final float c = MathUtils.cos(m_sweep.a), s = MathUtils.sin(m_sweep.a);
		m_xf.R.col1.x = c;
		m_xf.R.col2.x = -s;
		m_xf.R.col1.y = s;
		m_xf.R.col2.y = c;
		final Vec localCenter = m_sweep.localCenter;
		m_xf.position.x = m_sweep.c.x - m_xf.R.col1.x * localCenter.x
				- m_xf.R.col2.x * localCenter.y;
		m_xf.position.y = m_sweep.c.y - m_xf.R.col1.y * localCenter.x
				- m_xf.R.col2.y * localCenter.y;
	}

	/**
	 * This is used to prevent connected bodies from colliding. It may lie,
	 * depending on the collideConnected flag.
	 * 
	 * @param other
	 * @return
	 */
	public boolean shouldCollide(RigidBody other) {
		// At least one body should be dynamic.
		if (m_type != RigidBodyType.DYNAMIC && other.m_type != RigidBodyType.DYNAMIC) {
			return false;
		}

		return true;
	}

	protected final void advance(float t) {
		// Advance to the new safe time.
		m_sweep.advance(t);
		m_sweep.c.set(m_sweep.c0);
		m_sweep.a = m_sweep.a0;
		synchronizeTransform();
	}
}
