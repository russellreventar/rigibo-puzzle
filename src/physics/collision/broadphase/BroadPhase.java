package physics.collision.broadphase;

import java.util.Arrays;

import physics.collision.AABB;
import physics.collision.PairCallback;
import physics.collision.TreeCallback;
import physics.tools.Vec;

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray
 * casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 * 
 */
public class BroadPhase implements TreeCallback {
	
	public static final int NULL_PROXY = -1;
	
	private final DynamicTree m_tree;
	
	private int m_proxyCount;
	
	private DynamicTreeNode[] m_moveBuffer;
	private int m_moveCapacity;
	private int m_moveCount;
	
	private Pair[] m_pairBuffer;
	private int m_pairCapacity;
	private int m_pairCount;
	
	private DynamicTreeNode m_queryProxy;
	
	public BroadPhase() {
		m_proxyCount = 0;
		
		m_pairCapacity = 16;
		m_pairCount = 0;
		m_pairBuffer = new Pair[m_pairCapacity];
		for (int i = 0; i < m_pairCapacity; i++) {
			m_pairBuffer[i] = new Pair();
		}
		
		m_moveCapacity = 16;
		m_moveCount = 0;
		m_moveBuffer = new DynamicTreeNode[m_moveCapacity];
		
		m_tree = new DynamicTree();
		m_queryProxy = null;
	}
	
	/**
	 * Create a proxy with an initial AABB. Pairs are not reported until
	 * updatePairs is called.
	 * 
	 * @param aabb
	 * @param userData
	 * @return
	 */
	public final DynamicTreeNode createProxy(final AABB aabb, Object userData) {
		DynamicTreeNode node = m_tree.createProxy(aabb, userData);
		++m_proxyCount;
		bufferMove(node);
		return node;
	}
	
	/**
	 * Destroy a proxy. It is up to the client to remove any pairs.
	 * 
	 * @param proxyId
	 */
	public final void destroyProxy(DynamicTreeNode proxy) {
		unbufferMove(proxy);
		--m_proxyCount;
		m_tree.destroyProxy(proxy);
	}
	
	/**
	 * Call MoveProxy as many times as you like, then when you are done
	 * call UpdatePairs to finalized the proxy pairs (for your time step).
	 */
	public final void moveProxy(DynamicTreeNode proxy, final AABB aabb, final Vec displacement) {
		boolean buffer = m_tree.moveProxy(proxy, aabb, displacement);
		if (buffer) {
			bufferMove(proxy);
		}
	}
	
	public boolean testOverlap(DynamicTreeNode proxyA, DynamicTreeNode proxyB) {
		AABB a = proxyA.aabb;
		AABB b = proxyB.aabb;
		if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f) {
			return false;
		}
		
		if (a.lowerBound.x - b.upperBound.x > 0.0f || a.lowerBound.y - b.upperBound.y > 0.0f) {
			return false;
		}
		
		return true;
	}
	
	/**
	 * Get the number of proxies.
	 * 
	 * @return
	 */
	public final int getProxyCount() {
		return m_proxyCount;
	}
	
	
	/**
	 * Update the pairs. This results in pair callbacks. This can only add pairs.
	 * 
	 * @param callback
	 */
	public final void updatePairs(PairCallback callback) {
		// log.debug("beginning to update pairs");
		// Reset pair buffer
		m_pairCount = 0;
		
		// Perform tree queries for all moving proxies.
		for (int i = 0; i < m_moveCount; ++i) {
			m_queryProxy = m_moveBuffer[i];
			if (m_queryProxy == null) {
				continue;
			}
			
			// We have to query the tree with the fat AABB so that
			// we don't fail to create a pair that may touch later.
			// final AABB fatAABB = m_tree.getFatAABB(m_queryProxy);
			
			// Query tree, create pairs and add them pair buffer.

			m_tree.query(this, m_queryProxy.aabb);
		}

		
		// Reset move buffer
		m_moveCount = 0;
		
		// Sort the pair buffer to expose duplicates.
		Arrays.sort(m_pairBuffer, 0, m_pairCount);
		
		// Send the pairs back to the client.
		int i = 0;
		while (i < m_pairCount) {
			Pair primaryPair = m_pairBuffer[i];
			Object userDataA = m_tree.getUserData(primaryPair.proxyIdA);
			Object userDataB = m_tree.getUserData(primaryPair.proxyIdB);
			
			// log.debug("returning pair: "+userDataA+", "+userDataB);
			callback.addPair(userDataA, userDataB);
			++i;
			
			// Skip any duplicate pairs.
			while (i < m_pairCount) {
				Pair pair = m_pairBuffer[i];
				if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB) {
					break;
				}
				// log.debug("skipping duplicate");
				++i;
			}
		}
	}
	
	/**
	 * Query an AABB for overlapping proxies. The callback class
	 * is called for each proxy that overlaps the supplied AABB.
	 * 
	 * @param callback
	 * @param aabb
	 */
	public final void query(final TreeCallback callback, final AABB aabb) {
		m_tree.query(callback, aabb);
	}
	
	/**
	 * Compute the height of the embedded tree.
	 * 
	 * @return
	 */
	public final int computeHeight() {
		return m_tree.computeHeight();
	}
	
	protected final void bufferMove(DynamicTreeNode node) {
		if (m_moveCount == m_moveCapacity) {
			DynamicTreeNode[] old = m_moveBuffer;
			m_moveCapacity *= 2;
			m_moveBuffer = new DynamicTreeNode[m_moveCapacity];
			for (int i = 0; i < old.length; i++) {
				m_moveBuffer[i] = old[i];
			}
		}
		
		m_moveBuffer[m_moveCount] = node;
		++m_moveCount;
	}
	
	protected final void unbufferMove(DynamicTreeNode proxy) {
		for (int i = 0; i < m_moveCount; i++) {
			if (m_moveBuffer[i] == proxy) {
				m_moveBuffer[i] = null;
			}
		}
	}
	
	// private final PairStack pairStack = new PairStack();
	/**
	 * This is called from DynamicTree::query when we are gathering pairs.
	 */
	public final boolean treeCallback(DynamicTreeNode proxy) {
		
		// log.debug("Got a proxy back");
		// A proxy cannot form a pair with itself.
		if (proxy == m_queryProxy) {
			// log.debug("It was us...");
			return true;
		}
		
		// Grow the pair buffer as needed.
		if (m_pairCount == m_pairCapacity) {
			Pair[] oldBuffer = m_pairBuffer;
			m_pairCapacity *= 2;
			m_pairBuffer = new Pair[m_pairCapacity];
			for (int i = 0; i < oldBuffer.length; i++) {
				m_pairBuffer[i] = oldBuffer[i];
			}
			for (int i = oldBuffer.length; i < m_pairCapacity; i++) {
				m_pairBuffer[i] = new Pair();
			}
		}
		
		if (proxy.id < m_queryProxy.id) {
			m_pairBuffer[m_pairCount].proxyIdA = proxy.id;
			m_pairBuffer[m_pairCount].proxyIdB = m_queryProxy.id;
		}
		else {
			m_pairBuffer[m_pairCount].proxyIdA = m_queryProxy.id;
			m_pairBuffer[m_pairCount].proxyIdB = proxy.id;
		}
		
		++m_pairCount;
		return true;
	}
}
