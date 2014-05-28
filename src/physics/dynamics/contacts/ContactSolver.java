package physics.dynamics.contacts;


import physics.collision.Manifold;
import physics.collision.ManifoldPoint;
import physics.collision.WorldManifold;
import physics.collision.shapes.Shape;
import physics.dynamics.RigidBody;
import physics.dynamics.Fixture;
import physics.tools.Mat22;
import physics.tools.MathUtils;
import physics.tools.Settings;
import physics.tools.Vec;

public class ContactSolver {
	
	/**
	 * For each solver, this is the initial number of constraints in the array, which expands
	 * as needed.
	 */
	public static final int INITIAL_NUM_CONSTRAINTS = 256;
	
	/**
	 * Ensure a reasonable condition number. for the block solver
	 */
	public static final float k_maxConditionNumber = 100.0f;
	
	public ContactConstraint[] m_constraints;
	public int m_constraintCount;
	
	public ContactSolver(){
		m_constraints = new ContactConstraint[INITIAL_NUM_CONSTRAINTS];
		for(int i=0; i< m_constraints.length; i++){
			m_constraints[i] = new ContactConstraint();
		}
	}
	
	// djm pooling
	private final WorldManifold worldManifold = new WorldManifold();
	private final Vec tangent = new Vec();
	private final Vec temp1 = new Vec();
	private final Vec temp2 = new Vec();
	
	public final void init(Contact[] contacts, int contactCount, float impulseRatio){

		m_constraintCount = contactCount;
		
		// dynamic array
		if(m_constraints.length < contactCount){
		  final ContactConstraint[] old = m_constraints;
		  final int newLen = MathUtils.max(old.length * 2, m_constraintCount);
		  m_constraints = new ContactConstraint[newLen];
		  System.arraycopy(old, 0, m_constraints, 0, old.length);
		  
			for(int i=old.length; i< m_constraints.length; i++){
					m_constraints[i] = new ContactConstraint();
			}
		}
		
		for (int i = 0; i < m_constraintCount; ++i){
			final Contact contact = contacts[i];

			final Fixture fixtureA = contact.m_fixtureA;
			final Fixture fixtureB = contact.m_fixtureB;
			final Shape shapeA = fixtureA.getShape();
			final Shape shapeB = fixtureB.getShape();
			final float radiusA = shapeA.m_radius;
			final float radiusB = shapeB.m_radius;
			final RigidBody bodyA = fixtureA.getBody();
			final RigidBody bodyB = fixtureB.getBody();
			final Manifold manifold = contact.getManifold();

			final float friction = Settings.mixFriction(fixtureA.getFriction(), fixtureB.getFriction());
			final float restitution = Settings.mixRestitution(fixtureA.getRestitution(), fixtureB.getRestitution());

			final Vec vA = bodyA.m_linearVelocity;
			final Vec vB = bodyB.m_linearVelocity;
			final float wA = bodyA.m_angularVelocity;
			final float wB = bodyB.m_angularVelocity;

			assert(manifold.pointCount > 0);

			worldManifold.initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);

			final ContactConstraint cc = m_constraints[i];
			cc.bodyA = bodyA;
			cc.bodyB = bodyB;
			cc.manifold = manifold;
			cc.normal.x = worldManifold.normal.x;
			cc.normal.y = worldManifold.normal.y; // have to set actual manifold
			cc.pointCount = manifold.pointCount;
			cc.friction = friction;
			cc.restitution = restitution;
			cc.localNormal.x = manifold.localNormal.x;
			cc.localNormal.y = manifold.localNormal.y;
			cc.localPoint.x = manifold.localPoint.x;
			cc.localPoint.y = manifold.localPoint.y;
			cc.radius = radiusA + radiusB;
			cc.type = manifold.type;

			for (int j = 0; j < cc.pointCount; ++j){
				final ManifoldPoint cp = manifold.points[j];
				final ContactConstraintPoint ccp = cc.points[j];

				ccp.normalImpulse = impulseRatio * cp.normalImpulse;
				ccp.tangentImpulse = impulseRatio * cp.tangentImpulse;
				ccp.localPoint.x = cp.localPoint.x;
				ccp.localPoint.y = cp.localPoint.y;
				
				ccp.rA.x = worldManifold.points[j].x - bodyA.m_sweep.c.x;
				ccp.rA.y = worldManifold.points[j].y - bodyA.m_sweep.c.y;

				ccp.rB.x = worldManifold.points[j].x - bodyB.m_sweep.c.x;
				ccp.rB.y = worldManifold.points[j].y - bodyB.m_sweep.c.y;
				float rnA = ccp.rA.x * cc.normal.y - ccp.rA.y * cc.normal.x;
				float rnB = ccp.rB.x * cc.normal.y - ccp.rB.y * cc.normal.x;
				rnA *= rnA;
				rnB *= rnB;

				final float kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;

				assert(kNormal > Settings.EPSILON);
				ccp.normalMass = 1.0f / kNormal;
				
				tangent.x = 1.0f * cc.normal.y;
				tangent.y = -1.0f * cc.normal.x;
				
				float rtA = ccp.rA.x * tangent.y - ccp.rA.y * tangent.x;
				float rtB = ccp.rB.x * tangent.y - ccp.rB.y * tangent.x;
				rtA *= rtA;
				rtB *= rtB;

				final float kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;

				assert(kTangent > Settings.EPSILON);
				ccp.tangentMass = 1.0f /  kTangent;

				// Setup a velocity bias for restitution.
				ccp.velocityBias = 0.0f;
				temp2.x = -wA * ccp.rA.y;
				temp2.y = wA * ccp.rA.x;
				//temp1.addLocal(vB).subLocal(vA).subLocal(temp2);
				temp1.x = -wB * ccp.rB.y + vB.x - vA.x - temp2.x;
				temp1.y = wB * ccp.rB.x + vB.y - vA.y - temp2.y;
				final Vec a = cc.normal;

				
				//float vRel = Dot(cc.normal, vB + Cross(wB, ccp.rB) - vA - Cross(wA, ccp.rA));
				final float vRel = a.x * temp1.x + a.y * temp1.y;
				
				if (vRel < -Settings.velocityThreshold){
					ccp.velocityBias = -restitution * vRel;
				}
			}

			// If we have two points, then prepare the block solver.
			if (cc.pointCount == 2){
				final ContactConstraintPoint ccp1 = cc.points[0];
				final ContactConstraintPoint ccp2 = cc.points[1];
				
				final float invMassA = bodyA.m_invMass;
				final float invIA = bodyA.m_invI;
				final float invMassB = bodyB.m_invMass;
				final float invIB = bodyB.m_invI;

				final float rn1A = Vec.cross(ccp1.rA, cc.normal);
				final float rn1B = Vec.cross(ccp1.rB, cc.normal);
				final float rn2A = Vec.cross(ccp2.rA, cc.normal);
				final float rn2B = Vec.cross(ccp2.rB, cc.normal);

				final float k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
				final float k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
				final float k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				//final float k_maxConditionNumber = 100.0f;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)){
					// K is safe to invert.
					cc.K.col1.x = k11;
					cc.K.col1.y = k12;
					cc.K.col2.x = k12;
					cc.K.col2.y = k22;
					cc.normalMass.col1.x = cc.K.col1.x;
					cc.normalMass.col1.y = cc.K.col1.y;
					cc.normalMass.col2.x = cc.K.col2.x;
					cc.normalMass.col2.y = cc.K.col2.y;
					cc.normalMass.invertLocal();
				}
				else{
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					cc.pointCount = 1;
				}
			}
		}
	}
		
	public void warmStart(){
		// Warm start.
		for (int i = 0; i < m_constraintCount; ++i){
			final ContactConstraint c = m_constraints[i];

			final RigidBody bodyA = c.bodyA;
			final RigidBody bodyB = c.bodyB;
			final float invMassA = bodyA.m_invMass;
			final float invIA = bodyA.m_invI;
			final float invMassB = bodyB.m_invMass;
			final float invIB = bodyB.m_invI;
			final Vec normal = c.normal;
			Vec.crossToOut(normal, 1f, tangent);

			for (int j = 0; j < c.pointCount; ++j){
				final ContactConstraintPoint ccp = c.points[j];				
				
				final float Px = ccp.normalImpulse * normal.x + ccp.tangentImpulse * tangent.x;
				final float Py = ccp.normalImpulse * normal.y + ccp.tangentImpulse * tangent.y;

				bodyA.m_angularVelocity -= invIA * (ccp.rA.x * Py - ccp.rA.y * Px);
				bodyA.m_linearVelocity.x -= Px * invMassA;
				bodyA.m_linearVelocity.y -= Py * invMassA;
				
				bodyB.m_angularVelocity += invIB * (ccp.rB.x * Py - ccp.rB.y * Px);
				bodyB.m_linearVelocity.x += Px * invMassB;
				bodyB.m_linearVelocity.y += Py * invMassB;
			}
		}
	}
	
	//djm pooling from above
	private final Vec dv = new Vec();
	private final Vec a = new Vec();
	private final Vec b = new Vec();
	private final Vec dv1 = new Vec();
	private final Vec dv2 = new Vec();
	private final Vec x = new Vec();
	private final Vec d = new Vec();
	private final Vec P1 = new Vec();
	private final Vec P2 = new Vec();
	
	public final void solveVelocityConstraints(){
		for (int i = 0; i < m_constraintCount; ++i){
			final ContactConstraint c = m_constraints[i];
			final RigidBody bodyA = c.bodyA;
			final RigidBody bodyB = c.bodyB;
			float wA = bodyA.m_angularVelocity;
			float wB = bodyB.m_angularVelocity;
			final Vec vA = bodyA.m_linearVelocity;
			final Vec vB = bodyB.m_linearVelocity;
			final float invMassA = bodyA.m_invMass;
			final float invIA = bodyA.m_invI;
			final float invMassB = bodyB.m_invMass;
			final float invIB = bodyB.m_invI;
			tangent.x = 1.0f * c.normal.y;
			tangent.y = -1.0f * c.normal.x;
			final float friction = c.friction;

			assert(c.pointCount == 1 || c.pointCount == 2);

			// Solve tangent constraints
			for (int j = 0; j < c.pointCount; ++j){
				final ContactConstraintPoint ccp = c.points[j];
				final Vec a = ccp.rA;

				dv.x = -wB * ccp.rB.y + vB.x - vA.x + wA * a.y;
				dv.y = wB * ccp.rB.x + vB.y - vA.y - wA * a.x;

				// Compute tangent force
				final float vt = dv.x * tangent.x + dv.y * tangent.y;
				float lambda = ccp.tangentMass * (-vt);

				// Clamp the accumulated force
				final float maxFriction = friction * ccp.normalImpulse;
				final float newImpulse = MathUtils.clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - ccp.tangentImpulse;
				

				// Apply contact impulse
				//Vec2 P = lambda * tangent;
				
				final float Px = tangent.x * lambda;
				final float Py = tangent.y * lambda;
				
				//vA -= invMassA * P;
				vA.x -= Px * invMassA;
				vA.y -= Py * invMassA;
				wA -= invIA * (ccp.rA.x * Py - ccp.rA.y * Px);

				//vB += invMassB * P;
				vB.x += Px * invMassB;
				vB.y += Py * invMassB;
				wB += invIB * (ccp.rB.x * Py - ccp.rB.y * Px);

				ccp.tangentImpulse = newImpulse;
			}

			// Solve normal constraints
			if (c.pointCount == 1){
				final ContactConstraintPoint ccp = c.points[0];
				Vec a1 = ccp.rA;

				dv.x = -wB * ccp.rB.y + vB.x - vA.x + wA * a1.y;
				dv.y = wB * ccp.rB.x + vB.y - vA.y - wA * a1.x;
				Vec b = c.normal;

				// Compute normal impulse
				final float vn = dv.x * b.x + dv.y * b.y;
				float lambda = -ccp.normalMass * (vn - ccp.velocityBias);

				// Clamp the accumulated impulse
				float a = ccp.normalImpulse + lambda;
				final float newImpulse = (a > 0.0f ? a : 0.0f);
				lambda = newImpulse - ccp.normalImpulse;

				// Apply contact impulse
				float Px = c.normal.x * lambda;
				float Py = c.normal.y * lambda;
				
				//vA -= invMassA * P;
				vA.x -= Px * invMassA;
				vA.y -= Py * invMassA;
				wA -= invIA * (ccp.rA.x * Py - ccp.rA.y * Px);

				//vB += invMassB * P;
				vB.x += Px * invMassB;
				vB.y += Py * invMassB;
				wB += invIB * (ccp.rB.x * Py - ccp.rB.y * Px);
				
				ccp.normalImpulse = newImpulse;
			}
			else
			{
				final ContactConstraintPoint cp1 = c.points[0];
				final ContactConstraintPoint cp2 = c.points[1];
				a.x = cp1.normalImpulse;
				a.y = cp2.normalImpulse;

				assert(a.x >= 0.0f && a.y >= 0.0f);
				// Relative velocity at contact
				//Vec2 dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
				dv1.x = -wB * cp1.rB.y + vB.x - vA.x + wA * cp1.rA.y;
				dv1.y = wB * cp1.rB.x + vB.y - vA.y - wA * cp1.rA.x;
				
				//Vec2 dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
				dv2.x = -wB * cp2.rB.y + vB.x - vA.x + wA * cp2.rA.y;
				dv2.y = wB * cp2.rB.x + vB.y - vA.y - wA * cp2.rA.x;
				
				// Compute normal velocity
				float vn1 = dv1.x * c.normal.x + dv1.y * c.normal.y;
				float vn2 = dv2.x * c.normal.x + dv2.y * c.normal.y;

				b.x = vn1 - cp1.velocityBias;
				b.y = vn2 - cp2.velocityBias;
				temp2.x = c.K.col1.x * a.x + c.K.col2.x * a.y;
				temp2.y = c.K.col1.y * a.x + c.K.col2.y * a.y;
				b.x -= temp2.x;
				b.y -= temp2.y;

				//final float k_errorTol = 1e-3f;
				//B2_NOT_USED(k_errorTol);

				for (;;)
				{
					//
					// Case 1: vn = 0
					//
					// 0 = A * x' + b'
					//
					// Solve for x':
					//
					// x' = - inv(A) * b'
					//
					//Vec2 x = - Mul(c.normalMass, b);
					final Mat22 R = c.normalMass;
					// R.mulToOut(v,out);
					x.x = - R.col1.x * b.x - R.col2.x * b.y;
					x.y = - R.col1.y * b.x - R.col2.y * b.y;
					
					if (x.x >= 0.0f && x.y >= 0.0f){
						// Resubstitute for the incremental impulse
						//Vec2 d = x - a;
						d.set(x).subLocal(a);
						
						// Apply incremental impulse
						//Vec2 P1 = d.x * normal;
						//Vec2 P2 = d.y * normal;
						P1.set(c.normal).mulLocal(d.x);
						P2.set(c.normal).mulLocal(d.y);
						
						/*vA -= invMassA * (P1 + P2);
						wA -= invIA * (b2Cross(cp1->rA, P1) + b2Cross(cp2->rA, P2));
	
						vB += invMassB * (P1 + P2);
						wB += invIB * (b2Cross(cp1->rB, P1) + b2Cross(cp2->rB, P2));*/
						
						vA.x -= invMassA * (P1.x + P2.x);
						vA.y -= invMassA * (P1.y + P2.y);
						vB.x += invMassB * (P1.x + P2.x);
						vB.y += invMassB * (P1.y + P2.y);
												
						wA -= invIA * (Vec.cross(cp1.rA, P1) + Vec.cross(cp2.rA, P2));
						wB += invIB * (Vec.cross(cp1.rB, P1) + Vec.cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

	/*#if B2_DEBUG_SOLVER == 1
						// Postconditions
						dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
						dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);

						// Compute normal velocity
						vn1 = Dot(dv1, normal);
						vn2 = Dot(dv2, normal);

						assert(Abs(vn1 - cp1.velocityBias) < k_errorTol);
						assert(Abs(vn2 - cp2.velocityBias) < k_errorTol);
	#endif*/
						break;
					}

					//
					// Case 2: vn1 = 0 and x2 = 0
					//
					//   0 = a11 * x1' + a12 * 0 + b1' 
					// vn2 = a21 * x1' + a22 * 0 + '
					//
					x.x = - cp1.normalMass * b.x;
					x.y = 0.0f;
					vn1 = 0.0f;
					vn2 = c.K.col1.y * x.x + b.y;

					if (x.x >= 0.0f && vn2 >= 0.0f)
					{
						// Resubstitute for the incremental impulse
						final float dx = x.x - a.x;
						final float dy = x.y - a.y;

						P1.set(c.normal).mulLocal(dx);
						P2.set(c.normal).mulLocal(dy);
						
						vA.x -= invMassA * (P1.x + P2.x);
						vA.y -= invMassA * (P1.y + P2.y);
						vB.x += invMassB * (P1.x + P2.x);
						vB.y += invMassB * (P1.y + P2.y);
												
						wA -= invIA * (Vec.cross(cp1.rA, P1) + Vec.cross(cp2.rA, P2));
						wB += invIB * (Vec.cross(cp1.rB, P1) + Vec.cross(cp2.rB, P2));
						
						
						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						break;
					}

					x.x = 0.0f;
					x.y = - cp2.normalMass * b.y;
					vn1 = c.K.col2.x * x.y + b.x;
					vn2 = 0.0f;

					if (x.y >= 0.0f && vn1 >= 0.0f)
					{
						// Resubstitute for the incremental impulse
						final float dx = x.x - a.x;
						final float dy = x.y - a.y;
						
						P1.set(c.normal).mulLocal(dx);
						P2.set(c.normal).mulLocal(dy);
						
						vA.x -= invMassA * (P1.x + P2.x);
						vA.y -= invMassA * (P1.y + P2.y);
						vB.x += invMassB * (P1.x + P2.x);
						vB.y += invMassB * (P1.y + P2.y);
												
						wA -= invIA * (Vec.cross(cp1.rA, P1) + Vec.cross(cp2.rA, P2));
						wB += invIB * (Vec.cross(cp1.rB, P1) + Vec.cross(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						break;
					}

					x.x = 0.0f;
					x.y = 0.0f;
					vn1 = b.x;
					vn2 = b.y;

					if (vn1 >= 0.0f && vn2 >= 0.0f )
					{
						// Resubstitute for the incremental impulse
						final float dx = x.x - a.x;
						final float dy = x.y - a.y;

						P1.set(c.normal).mulLocal(dx);
						P2.set(c.normal).mulLocal(dy);
						
						vA.x -= invMassA * (P1.x + P2.x);
						vA.y -= invMassA * (P1.y + P2.y);
						vB.x += invMassB * (P1.x + P2.x);
						vB.y += invMassB * (P1.y + P2.y);
												
						wA -= invIA * (Vec.cross(cp1.rA, P1) + Vec.cross(cp2.rA, P2));
						wB += invIB * (Vec.cross(cp1.rB, P1) + Vec.cross(cp2.rB, P2));
						
						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						break;
					}

					// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
					break;
				}
			}

			bodyA.m_angularVelocity = wA;
			bodyB.m_angularVelocity = wB;
		}
	}
	
	public void storeImpulses(){
		for( int i=0; i<m_constraintCount; i++){
			final ContactConstraint c = m_constraints[i];
			final Manifold m = c.manifold;
			
			for(int j=0; j< c.pointCount; j++){
				m.points[j].normalImpulse = c.points[j].normalImpulse;
				m.points[j].tangentImpulse = c.points[j].tangentImpulse;
			}
		}
	}

	private final PositionSolverManifold psolver = new PositionSolverManifold();
	private final Vec rA = new Vec();
	private final Vec rB = new Vec();
	
	/**
	 * Sequential solver.
	 */
	public final boolean solvePositionConstraints(float baumgarte){
		float minSeparation = 0.0f;

		for (int i = 0; i < m_constraintCount; ++i){
			final ContactConstraint c = m_constraints[i];
			final RigidBody bodyA = c.bodyA;
			final RigidBody bodyB = c.bodyB;

			final float invMassA = bodyA.m_mass * bodyA.m_invMass;
			final float invIA = bodyA.m_mass * bodyA.m_invI;
			final float invMassB = bodyB.m_mass * bodyB.m_invMass;
			final float invIB = bodyB.m_mass * bodyB.m_invI;

			// Solve normal constraints
			for (int j = 0; j < c.pointCount; ++j){
				final PositionSolverManifold psm = psolver;
				psm.initialize(c, j);
				final Vec normal = psm.normal;
				
				final Vec point = psm.point;
				final float separation = psm.separation;

				rA.set(point).subLocal(bodyA.m_sweep.c);
				rB.set(point).subLocal(bodyB.m_sweep.c);

				// Track max constraint error.
				minSeparation = MathUtils.min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				final float C = MathUtils.clamp(baumgarte * (separation + Settings.linearSlop), -Settings.maxLinearCorrection, 0.0f);

				// Compute the effective mass.
				final float rnA = Vec.cross(rA, normal);
				final float rnB = Vec.cross(rB, normal);
				final float K = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;
				
				// Compute normal impulse
				final float impulse = K > 0.0f ? - C / K : 0.0f;

				final float Px = normal.x * impulse;
				final float Py = normal.y * impulse;

				bodyA.m_sweep.c.x -= Px * invMassA;
				bodyA.m_sweep.c.y -= Py * invMassA;
				bodyA.m_sweep.a -= invIA * (rA.x * Py - rA.y * Px);
				bodyA.synchronizeTransform();

				bodyB.m_sweep.c.x += Px * invMassB;
				bodyB.m_sweep.c.y += Py * invMassB;
				bodyB.m_sweep.a += invIB * (rB.x * Py - rB.y * Px);
				bodyB.synchronizeTransform();
			}
		}

		// We can't expect minSpeparation >= -linearSlop because we don't
		// push the separation above -linearSlop.
		return minSeparation >= -1.5f * Settings.linearSlop;
	}
}

class PositionSolverManifold{
	
	public final Vec normal = new Vec();
	public final Vec point = new Vec();
	public float separation;
	
	// djm pooling
	private final Vec pointA = new Vec();
	private final Vec pointB = new Vec();
	private final Vec temp = new Vec();
	private final Vec planePoint = new Vec();
	private final Vec clipPoint = new Vec();
	
	public void initialize(ContactConstraint cc, int index){
		assert(cc.pointCount > 0);
		
		switch (cc.type){
			case CIRCLES:{
				cc.bodyA.getWorldPointToOut(cc.localPoint, pointA);
				cc.bodyB.getWorldPointToOut(cc.points[0].localPoint, pointB);
				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON){
					normal.set(pointB).subLocal(pointA);
					normal.normalize();
				}
				else{
					normal.set(1.0f, 0.0f);
				}

				point.set(pointA).addLocal(pointB).mulLocal(.5f);
				temp.set(pointB).subLocal(pointA);
				separation = Vec.dot(temp, normal) - cc.radius;
				break;
			}
	
			case FACE_A:{
				cc.bodyA.getWorldVectorToOut(cc.localNormal, normal);
				cc.bodyA.getWorldPointToOut(cc.localPoint, planePoint);

				cc.bodyB.getWorldPointToOut(cc.points[index].localPoint, clipPoint);
				temp.set(clipPoint).subLocal(planePoint);
				separation = Vec.dot(temp, normal) - cc.radius;
				point.set(clipPoint);
				break;
			}
	
			case FACE_B:
				{
					cc.bodyB.getWorldVectorToOut(cc.localNormal, normal);
					cc.bodyB.getWorldPointToOut(cc.localPoint, planePoint);
	
					cc.bodyA.getWorldPointToOut(cc.points[index].localPoint, clipPoint);
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
