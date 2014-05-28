package physics.collision.broadphase;

/**
 * Java note: at the "creation" of each node, a random key is given to
 * that node, and that's what we sort from.
 */
public class Pair implements Comparable<Pair> {
	public int proxyIdA;
	public int proxyIdB;
	
	public int compareTo(Pair pair2) {
		if (this.proxyIdA < pair2.proxyIdA) {
			return -1;
		}
		
		if (this.proxyIdA == pair2.proxyIdA) {
			
			if (proxyIdB < pair2.proxyIdB) {
				return -1;
			}
			else {
				if (proxyIdB == pair2.proxyIdB) {
					return 0;
				}
				else {
					return 1;
				}
			}
		}
		return 1;
	}
}
