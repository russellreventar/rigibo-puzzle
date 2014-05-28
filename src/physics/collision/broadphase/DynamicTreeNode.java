package physics.collision.broadphase;

import physics.collision.AABB;

public class DynamicTreeNode {
	public static final int NULL_NODE = -1;
	/**
	 * This is the fattened AABB
	 */
	public final AABB aabb = new AABB();
	
	public Object userData;
	
	protected int parent;
	
	protected int child1;
	protected int child2;
	protected int height;
	
	protected int id;
	
	public final boolean isLeaf() {
		return child1 == NULL_NODE;
	}
	
	public Object getUserData() {
		return userData;
	}
	
	public void setUserData(Object argData) {
		userData = argData;
	}
	
	/**
	 * Should never be constructed outside the engine
	 */
	protected DynamicTreeNode() {}
}
