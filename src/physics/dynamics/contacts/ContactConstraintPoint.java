package physics.dynamics.contacts;

import physics.tools.Vec;

public class ContactConstraintPoint {
	
    public final Vec localPoint;
    public final Vec rA;
    public final Vec rB;

    public float normalImpulse;
    public float tangentImpulse;
    public float normalMass;
    public float tangentMass;
    public float velocityBias;

    public ContactConstraintPoint() {
        localPoint = new Vec();
        rA = new Vec();
        rB = new Vec();
    }
    
    public void set(final ContactConstraintPoint cp){
    	localPoint.set(cp.localPoint);
    	rA.set(cp.rA);
    	rB.set(cp.rB);
    	normalImpulse = cp.normalImpulse;
    	tangentImpulse = cp.tangentImpulse;
    	normalMass = cp.normalMass;
    	tangentMass = cp.tangentMass;
        velocityBias = cp.velocityBias;
    }
}
