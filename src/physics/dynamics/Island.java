package physics.dynamics;

import physics.collision.ContactListener;
import physics.dynamics.contacts.Contact;
import physics.dynamics.contacts.ContactSolver;
import physics.tools.MathUtils;
import physics.tools.Settings;
import physics.tools.Vec;

/**
 * This is an internal class.
 */
public class Island {
	
	public ContactListener m_listener;

	public RigidBody[] m_bodies;
	public Contact[] m_contacts;

	public Position[] m_positions;
	public Velocity[] m_velocities;
	
	public int m_bodyCount;
	public int m_jointCount;
	public int m_contactCount;
	
	public int m_bodyCapacity;
	public int m_contactCapacity;
	public int m_jointCapacity;
	
	public int m_positionIterationCount;
	
	public Island(){
		
	}
	
	public void init(int bodyCapacity, int contactCapacity, int jointCapacity, ContactListener listener){
		m_bodyCapacity = bodyCapacity;
		m_contactCapacity = contactCapacity;
		m_jointCapacity = jointCapacity;
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
		
		m_listener = listener;
		
		if(m_bodies == null || m_bodyCapacity > m_bodies.length){
			m_bodies = new RigidBody[m_bodyCapacity];
		}

		if(m_contacts == null || m_contactCapacity > m_contacts.length){
			m_contacts = new Contact[m_contactCapacity];
		}
		
		// dynamic array
		if(m_velocities == null || m_bodyCapacity > m_velocities.length){
		  final Velocity[] old = m_velocities == null ? new Velocity[0] : m_velocities;
			m_velocities = new Velocity[m_bodyCapacity];
			System.arraycopy(old, 0, m_velocities, 0, old.length);
			for(int i=old.length; i<m_velocities.length; i++){
				m_velocities[i] = new Velocity();
			}
		}
		
		// dynamic array
		if(m_positions == null || m_bodyCapacity > m_positions.length){
		  final Position[] old = m_positions == null ? new Position[0] : m_positions;
			m_positions = new Position[m_bodyCapacity];
			System.arraycopy(old, 0, m_positions, 0, old.length);
			for(int i=old.length; i<m_positions.length; i++){
				m_positions[i] = new Position();
			}
		}
	}
	
	public void clear(){
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}
	
	private final ContactSolver contactSolver = new ContactSolver();
	private final Vec translation = new Vec();
	
	public void solve(TimeStep step, Vec gravity, boolean allowSleep){
		// Integrate velocities and apply damping.
		for (int i = 0; i < m_bodyCount; ++i){
			RigidBody b = m_bodies[i];

			if (b.getType() != RigidBodyType.DYNAMIC){
				continue;
			}
			
			b.m_linearVelocity.x += (b.m_force.x * b.m_invMass + gravity.x)*step.dt;
			b.m_linearVelocity.y += (b.m_force.y * b.m_invMass + gravity.y)*step.dt;
			b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
			

			
			float a = (1.0f - step.dt * b.m_linearDamping);
			float a1 = (0.0f > (a < 1.0f ? a : 1.0f) ? 0.0f : (a < 1.0f ? a : 1.0f));
			b.m_linearVelocity.x *= a1;
			b.m_linearVelocity.y *= a1;
			
			float a2 = (1.0f - step.dt * b.m_angularDamping);
			float b1 = (a2 < 1.0f ? a2 : 1.0f);
			b.m_angularVelocity *= 0.0f > b1 ? 0.0f : b1;
		}

		// Partition contacts so that contacts with static bodies are solved last.
		int i1 = -1;
		for (int i2 = 0; i2 < m_contactCount; ++i2){
			Fixture fixtureA = m_contacts[i2].getFixtureA();
			Fixture fixtureB = m_contacts[i2].getFixtureB();
			RigidBody bodyA = fixtureA.getBody();
			RigidBody bodyB = fixtureB.getBody();
			boolean nonStatic = bodyA.getType() != RigidBodyType.STATIC && bodyB.getType() != RigidBodyType.STATIC;
			if (nonStatic){
				++i1;
				//Swap(m_contacts[i1], m_contacts[i2]);
				Contact temp = m_contacts[i1];
				m_contacts[i1] = m_contacts[i2];
				m_contacts[i2] = temp;
			}
		}

		// Initialize velocity constraints.
		contactSolver.init(m_contacts, m_contactCount, step.dtRatio);
		contactSolver.warmStart();

		// Solve velocity constraints.
		for (int i = 0; i < step.velocityIterations; ++i){
			contactSolver.solveVelocityConstraints();
		}

		// Post-solve (store impulses for warm starting).
		contactSolver.storeImpulses();

		// Integrate positions.
		for (int i = 0; i < m_bodyCount; ++i){
			RigidBody b = m_bodies[i];

			if (b.getType() == RigidBodyType.STATIC){
				continue;
			}

			// Check for large velocities.
			translation.set(b.m_linearVelocity).mulLocal(step.dt);
			if (Vec.dot(translation, translation) > Settings.maxTranslationSquared){
				float ratio = Settings.maxTranslation / translation.length();
				b.m_linearVelocity.x *= ratio;
				b.m_linearVelocity.y *= ratio;
			}

			float rotation = step.dt * b.m_angularVelocity;
			if (rotation * rotation > Settings.maxRotationSquared)
			{
				float ratio = Settings.maxRotation / Math.abs(rotation);
				b.m_angularVelocity *= ratio;
			}

			// Store positions for continuous collision.
			b.m_sweep.c0.set(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;


			// Integrate
			//b.m_sweep.c += step.dt * b.m_linearVelocity;
			b.m_sweep.c.x += b.m_linearVelocity.x * step.dt;
			b.m_sweep.c.y += b.m_linearVelocity.y * step.dt;
			b.m_sweep.a += step.dt * b.m_angularVelocity;

			// Compute new transform
			b.synchronizeTransform();

			// Note: shapes are synchronized later.
		}

		// Iterate over constraints.
		for (int i = 0; i < step.positionIterations; ++i){
			boolean contactsOkay = contactSolver.solvePositionConstraints(Settings.contactBaumgarte);

			if (contactsOkay){
				// Exit early if the position errors are small.
				break;
			}
		}

		if (allowSleep){
			float minSleepTime = Float.MAX_VALUE;

			 float linTolSqr = Settings.linearSleepTolerance * Settings.linearSleepTolerance;
			 float angTolSqr = Settings.angularSleepTolerance * Settings.angularSleepTolerance;

			for (int i = 0; i < m_bodyCount; ++i){
				RigidBody b = m_bodies[i];
				if (b.getType() == RigidBodyType.STATIC){
					continue;
				}

				if ((b.m_flags & RigidBody.e_autoSleepFlag) == 0){
					b.m_sleepTime = 0.0f;
					minSleepTime = 0.0f;
				}
				Vec linVel = b.m_linearVelocity;

				if ((b.m_flags & RigidBody.e_autoSleepFlag) == 0 ||
					b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
					linVel.x * linVel.x + linVel.y * linVel.y > linTolSqr){
					b.m_sleepTime = 0.0f;
					minSleepTime = 0.0f;
				}
				else{
					b.m_sleepTime += step.dt;
					minSleepTime = MathUtils.min(minSleepTime, b.m_sleepTime);
				}
			}

			if (minSleepTime >= Settings.timeToSleep){
				for (int i = 0; i < m_bodyCount; ++i){
					RigidBody b = m_bodies[i];
					b.setAwake(false);
				}
			}
		}
	}
	
	public void add(RigidBody body){
		assert(m_bodyCount < m_bodyCapacity);
		body.m_islandIndex = m_bodyCount;
		m_bodies[m_bodyCount++] = body;
	}
	
	public void add(Contact contact){
		assert(m_contactCount < m_contactCapacity);
		m_contacts[m_contactCount++] = contact;
	}
}

/**
 * This is an internal structure
 */
class Position{
	final Vec x = new Vec();
	float a;
}

/**
 * This is an internal structure
 */
class Velocity{
	final Vec v = new Vec();
	float a;
}
