package physics.dynamics.contacts;


import physics.collision.Manifold.ManifoldType;
import physics.dynamics.RigidBody;
import physics.tools.Settings;
import physics.tools.Vec;

public class TOIConstraint {
	public final Vec localPoints[] = new Vec[Settings.maxManifoldPoints];
	public final Vec localNormal = new Vec();
	public final Vec localPoint = new Vec();
	public ManifoldType type;
	public float radius;
	public int pointCount;
	public RigidBody bodyA;
	public RigidBody bodyB;
	
	public TOIConstraint(){
		for(int i=0; i<localPoints.length; i++){
			localPoints[i] = new Vec();
		}
	}
	
	public TOIConstraint(TOIConstraint argToClone){
		this();
		set(argToClone);
	}
	
	public void set(TOIConstraint argOther){
		for(int i=0; i<localPoints.length; i++){
			localPoints[i].set(argOther.localPoints[i]);
		}
		localNormal.set(argOther.localNormal);
		localPoint.set(argOther.localPoint);
		type = argOther.type;
		radius = argOther.radius;
		pointCount = argOther.pointCount;
		bodyA = argOther.bodyA;
		bodyB = argOther.bodyB;
	}
}
