package physics.dynamics.contacts;


import physics.collision.Manifold;
import physics.collision.Manifold.ManifoldType;
import physics.dynamics.RigidBody;
import physics.tools.Mat22;
import physics.tools.Settings;
import physics.tools.Vec;

public class ContactConstraint {
	public final ContactConstraintPoint points[];

 	public final Vec localNormal;
    public final Vec localPoint;
    public final Vec normal;
    
    public final Mat22 normalMass;
    public final Mat22 K;

    public RigidBody bodyA;
    public RigidBody bodyB;

    public ManifoldType type;
    
    public float radius;
    public float friction;
    public float restitution;
    public int pointCount;

    public Manifold manifold = null;

    public ContactConstraint() {
        points = new ContactConstraintPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            points[i] = new ContactConstraintPoint();
        }
        pointCount = 0;
        localNormal = new Vec();
        localPoint = new Vec();
        normal = new Vec();
        normalMass = new Mat22();
        K = new Mat22();

    }
    
    public void set(final ContactConstraint cp){
    	pointCount = cp.pointCount;
    	localNormal.set(cp.localNormal);
    	localPoint.set(cp.localPoint);
    	normal.set(cp.normal);
    	normalMass.set(cp.normalMass);
    	K.set(cp.K);
    	bodyA = cp.bodyA;
    	bodyB = cp.bodyB;
    	type = cp.type;
    	radius = cp.radius;
    	friction = cp.friction;
    	restitution = cp.restitution;
    	manifold = cp.manifold; // djm: not copy here
    	for(int i=0; i<cp.pointCount; i++){
    		points[i].set(cp.points[i]);
    	}
    }
}
