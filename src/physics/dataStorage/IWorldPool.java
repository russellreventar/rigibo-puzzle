package physics.dataStorage;

import physics.collision.AABB;
import physics.collision.Collision;
import physics.collision.Distance;
import physics.collision.TimeOfImpact;
import physics.dynamics.contacts.Contact;
import physics.tools.Mat22;
import physics.tools.Vec;

/**
 * World pool interface
 *
 */
public interface IWorldPool {

	public IDynamicStack<Contact> getPolyContactStack();

	public IDynamicStack<Contact> getCircleContactStack();

	public IDynamicStack<Contact> getPolyCircleContactStack();

	public Vec popVec2();

	public Vec[] popVec2(int argNum);

	public void pushVec2(int argNum);

	public void pushVec3(int argNum);

	public Mat22 popMat22();

	public Mat22[] popMat22(int argNum);

	public void pushMat22(int argNum);

	public AABB popAABB();

	public AABB[] popAABB(int argNum);

	public void pushAABB(int argNum);

	public Collision getCollision();

	public TimeOfImpact getTimeOfImpact();

	public Distance getDistance();

	public float[] getFloatArray(int argLength);

	public int[] getIntArray(int argLength);

	public Vec[] getVec2Array(int argLength);

}
