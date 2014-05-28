package physics.collision;

import physics.collision.broadphase.DynamicTreeNode;

/**
 * callback for  DynamicTree}
 */
public interface TreeCallback {
	
	/**
	 * Callback from a query request.  
	 */
	public boolean treeCallback(DynamicTreeNode node);
}
