package physics.dynamics.contacts;

import physics.collision.Manifold;
import physics.collision.ManifoldPoint;
import physics.collision.shapes.Shape;
import physics.dynamics.RigidBody;
import physics.dynamics.Fixture;
import physics.tools.MathUtils;
import physics.tools.Settings;
import physics.tools.Vec;

/**
 * This is a pure position solver for a single movable body in contact with
 * multiple non-moving bodies.
 */
public class TOISolver {
	// TODO djm: find out the best number to start with
	private TOIConstraint[] m_constraints = new TOIConstraint[4];
	private int m_count;
	private RigidBody m_toiBody;
	
	public TOISolver(){
		m_count = 0;
		m_toiBody = null;
		for(int i=0; i<m_constraints.length; i++){
			m_constraints[i] = new TOIConstraint();
		}
	}
	
	public void clear(){
		// does nothing
	}
	
	public void initialize(Contact[] contacts, int count, RigidBody toiBody){
		//clear();
		
		m_count = count;
		m_toiBody = toiBody;
		
		if(m_count > m_constraints.length){
			TOIConstraint[] old = m_constraints;
			// thanks zel1990!  Fix for issue 24
			final int newLen = MathUtils.max(m_count, old.length*2);
			m_constraints = new TOIConstraint[newLen];
			System.arraycopy(old, 0, m_constraints, 0, old.length);
			for(int i=old.length; i<m_constraints.length; i++){
				m_constraints[i] = new TOIConstraint();
			}
		}

		for(int i=0; i<m_count; i++){
			Contact contact = contacts[i];
			
			Fixture fixtureA = contact.getFixtureA();
			Fixture fixtureB = contact.getFixtureB();
			Shape shapeA = fixtureA.getShape();
			Shape shapeB = fixtureB.getShape();
			float radiusA = shapeA.m_radius;
			float radiusB = shapeB.m_radius;
			RigidBody bodyA = fixtureA.getBody();
			RigidBody bodyB = fixtureB.getBody();
			Manifold manifold = contact.getManifold();
			
			assert(manifold.pointCount > 0);
			
			TOIConstraint constraint = m_constraints[i];
			constraint.bodyA = bodyA;
			constraint.bodyB = bodyB;
			constraint.localNormal.set(manifold.localNormal);
			constraint.localPoint.set(manifold.localPoint);
			constraint.type = manifold.type;
			constraint.pointCount = manifold.pointCount;
			constraint.radius = radiusA + radiusB;

			for (int j = 0; j < constraint.pointCount; ++j){
				ManifoldPoint cp = manifold.points[j];
				constraint.localPoints[j] = cp.localPoint;
			}
		}
	}
	
	// djm pooling
	private final TOISolverManifold psm = new TOISolverManifold();
	private final Vec rA = new Vec();
	private final Vec rB = new Vec();
	private final Vec P = new Vec();
	private final Vec temp = new Vec();
	/**
	 * Perform one solver iteration. Returns true if converged.
	 * @param baumgarte
	 * @return
	 */
	public boolean solve(float baumgarte){
		// Push out the toi body to provide clearance for further simulation.
		
		float minSeparation = 0f;
		
		for (int i = 0; i < m_count; ++i){
			TOIConstraint c = m_constraints[i];
			RigidBody bodyA = c.bodyA;
			RigidBody bodyB = c.bodyB;

			float massA = bodyA.m_mass;
			float massB = bodyB.m_mass;

			// Only the TOI body should move.
			if (bodyA == m_toiBody){
				massB = 0.0f;
			}
			else{
				massA = 0.0f;
			}

			float invMassA = massA * bodyA.m_invMass;
			float invIA = massA * bodyA.m_invI;
			float invMassB = massB * bodyB.m_invMass;
			float invIB = massB * bodyB.m_invI;

			// Solve normal constraints
			for (int j = 0; j < c.pointCount; ++j){
				psm.initialize(c, j);
				Vec normal = psm.normal;

				Vec point = psm.point;
				float separation = psm.separation;

				rA.set(point).subLocal(bodyA.m_sweep.c);
				rB.set(point).subLocal(bodyB.m_sweep.c);

				// Track max constraint error.
				minSeparation = MathUtils.min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				float C = MathUtils.clamp(baumgarte * (separation + Settings.linearSlop), -Settings.maxLinearCorrection, 0.0f);

				// Compute the effective mass.
				float rnA = Vec.cross(rA, normal);
				float rnB = Vec.cross(rB, normal);
				float K = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;

				// Compute normal impulse
				float impulse = K > 0.0f ? - C / K : 0.0f;

				P.set(normal).mulLocal(impulse);

				temp.set(P).mulLocal(invMassA);
				bodyA.m_sweep.c.subLocal(temp);
				bodyA.m_sweep.a -= invIA * Vec.cross(rA, P);
				bodyA.synchronizeTransform();

				temp.set(P).mulLocal(invMassB);
				bodyB.m_sweep.c.addLocal(temp);
				bodyB.m_sweep.a += invIB * Vec.cross(rB, P);
				bodyB.synchronizeTransform();
			}
		}

		// We can't expect minSpeparation >= -_linearSlop because we don't
		// push the separation above -_linearSlop.
		return minSeparation >= -1.5f * Settings.linearSlop;
	}
}

class TOISolverManifold{
	public final Vec normal = new Vec();
	public final Vec point = new Vec();
	public float separation;
	
	// djm pooling
	private final Vec pointA = new Vec();
	private final Vec pointB = new Vec();
	private final Vec temp = new Vec();
	private final Vec planePoint = new Vec();
	private final Vec clipPoint = new Vec();
	
	public void initialize(TOIConstraint cc, int index){
		assert(cc.pointCount > 0);

		switch (cc.type){
			case CIRCLES : {
				cc.bodyA.getWorldPointToOut(cc.localPoint, pointA);
				cc.bodyB.getWorldPointToOut(cc.localPoints[0], pointB);
				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
					normal.set(pointB).subLocal(pointA);
					normal.normalize();
				}
				else {
					normal.set(1.0f, 0.0f);
				}
				
				point.set(pointA).addLocal(pointB).mulLocal(.5f);
				temp.set(pointB).subLocal(pointA);
				separation = Vec.dot(temp, normal) - cc.radius;
				break;
			}
			case FACE_A : {
				cc.bodyA.getWorldVectorToOut(cc.localNormal, normal);
				cc.bodyA.getWorldPointToOut(cc.localPoint, planePoint);
				
				cc.bodyB.getWorldPointToOut(cc.localPoints[index], clipPoint);
				temp.set(clipPoint).subLocal(planePoint);
				separation = Vec.dot(temp, normal) - cc.radius;
				point.set(clipPoint);
				break;
			}
			
			case FACE_B : {
				cc.bodyB.getWorldVectorToOut(cc.localNormal, normal);
				cc.bodyB.getWorldPointToOut(cc.localPoint, planePoint);
				
				cc.bodyA.getWorldPointToOut(cc.localPoints[index], clipPoint);
				temp.set(clipPoint).subLocal(planePoint);
				separation = Vec.dot(temp, normal) - cc.radius;
				point.set(clipPoint);
				
				// Ensure normal points from A to B
				normal.negateLocal();
			}
				break;
		}
	}
}
