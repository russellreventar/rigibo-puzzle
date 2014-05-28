package physics.dynamics;

import physics.collision.AABB;
import physics.collision.broadphase.BroadPhase;
import physics.collision.broadphase.DynamicTreeNode;
import physics.collision.shapes.Mass;
import physics.collision.shapes.Shape;
import physics.collision.shapes.ShapeType;
import physics.tools.Transform;
import physics.tools.Vec;

/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture
 * inherits its transform from its parent. Fixtures hold additional non-geometric data
 * such as friction, collision filters, etc.
 * Fixtures are created via 
 * @warning you cannot reuse fixtures.
 *
 */
public class Fixture {

	public final AABB m_aabb = new AABB();
	
	public float m_density;
	
	public Fixture m_next;
	public RigidBody m_body;
	
	public Shape m_shape;
	
	public float m_friction;
	public float m_restitution;
	
	public DynamicTreeNode m_proxy;
	
	public boolean m_isSensor;
	
	public Object m_userData;
	
	public Fixture(){
		m_userData = null;
		m_body = null;
		m_next = null;
		m_proxy = null;
		m_shape = null;
	}
	
	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public ShapeType getType(){
		return m_shape.getType();
	}
	
	/**
	 * Get the child shape. You can modify the child shape, however you should not change the
	 * number of vertices because this will crash some collision caching mechanisms.
	 * @return
	 */
	public Shape getShape(){
		return m_shape;
	}
	/**
	 * Is this fixture a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 * @return
	 */
	public boolean isSensor(){
		return m_isSensor;
	}
	
	/**
	 * Set if this fixture is a sensor.
	 * @param sensor
	 */
	public void setSensor(boolean sensor){
		m_isSensor = sensor;
	}
	
	/**
	 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
	 * @return the parent body.
	 * @return
	 */
	public RigidBody getBody(){
		return m_body;
	}
	
	/**
	 * Get the next fixture in the parent body's fixture list.
	 * @return the next shape.
	 * @return
	 */
	public Fixture getNext(){
		return m_next;
	}
	
	public void setDensity(float density){
		assert(density >= 0f);
		m_density = density;
	}
	
	public float getDensity(){
		return m_density;
	}
	
	/**
	 * Get the user data that was assigned in the fixture definition. Use this to
	 * store your application specific data.
	 * @return
	 */
	public Object getUserData(){
		return m_userData;
	}

	/**
	 * Set the user data. Use this to store your application specific data.
	 * @param data
	 */
	public void setUserData(Object data){
		m_userData = data;
	}
	
	/**
	 * Test a point for containment in this fixture. This only works for convex shapes.
	 * @param xf the shape world transform.
	 * @param p a point in world coordinates.
	 * @param p
	 * @return
	 */
	public boolean testPoint(final Vec p){
		return m_shape.testPoint( m_body.m_xf, p);
	}
	
	/**
	 * Get the mass data for this fixture. The mass data is based on the density and
	 * the shape. The rotational inertia is about the shape's origin.
	 * @return
	 */
	public void getMassData(Mass massData){
		m_shape.computeMass(massData, m_density);
	}

	/**
	 * Get the coefficient of friction.
	 * @return
	 */
	public float getFriction(){
		return m_friction;
	}

	/**
	 * Set the coefficient of friction.
	 * @param friction
	 */
	public void setFriction(float friction){
		m_friction = friction;
	}

	/**
	 * Get the coefficient of restitution.
	 * @return
	 */
	public float getRestitution(){
		return m_restitution;
	}

	/**
	 * Set the coefficient of restitution.
	 * @param restitution
	 */
	public void setRestitution(float restitution){
		m_restitution = restitution;
	}
	
	/**
	 * Get the fixture's AABB. This AABB may be enlarge and/or stale.
	 * If you need a more accurate AABB, compute it using the shape and
	 * the body transform.
	 * @return
	 */
	public AABB getAABB(){
		return m_aabb;
	}
	
	
	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by C++).
	
	public void create(RigidBody body, FixtureDef def){
		m_userData = def.userData;
		m_friction = def.friction;
		m_restitution = def.restitution;
		
		m_body = body;
		m_next = null;
		
		m_isSensor = def.isSensor;
		
		m_shape = def.shape.clone();
		
		m_density = def.density;
	}
	
	public void destroy(){
		
		// The proxy must be destroyed before calling this.
		assert(m_proxy == null);
		
		// Free the child shape.
		// yeah woo jvm
		// TODO djm should I pool this then?
		m_shape = null;
	}
	
	// These support body activation/deactivation.
	public void createProxy(BroadPhase broadPhase, final Transform xf){
		assert(m_proxy == null);
		
		// Create proxy in the broad-phase.
		m_shape.computeAABB( m_aabb, xf);
		m_proxy = broadPhase.createProxy( m_aabb, this);
	}
	
	/**
	 * Internal method
	 * @param broadPhase
	 */
	public void destroyProxy(BroadPhase broadPhase){
		if(m_proxy == null){
			return;
		}
		
		broadPhase.destroyProxy( m_proxy);
		m_proxy = null;
	}
	
	private final AABB pool1 = new AABB();
	private final AABB pool2 = new AABB();
	
	/**
	 * Internal method
	 * @param broadPhase
	 * @param xf1
	 * @param xf2
	 */
	protected void synchronize(BroadPhase broadPhase, final Transform transform1, final Transform transform2){
		if(m_proxy == null){
			return;
		}
		
		m_shape.computeAABB( pool1, transform1);
		m_shape.computeAABB( pool2, transform2);
		m_aabb.lowerBound.x = pool1.lowerBound.x < pool2.lowerBound.x ? pool1.lowerBound.x : pool2.lowerBound.x;
		m_aabb.lowerBound.y = pool1.lowerBound.y < pool2.lowerBound.y ? pool1.lowerBound.y : pool2.lowerBound.y;
		m_aabb.upperBound.x = pool1.upperBound.x > pool2.upperBound.x ? pool1.upperBound.x : pool2.upperBound.x;
		m_aabb.upperBound.y = pool1.upperBound.y > pool2.upperBound.y ? pool1.upperBound.y : pool2.upperBound.y;
		
		final Vec disp = pool1.lowerBound; // just use this vec for pooling
		disp.x = transform2.position.x - transform1.position.x;
		disp.y = transform2.position.y - transform1.position.y;
		
		broadPhase.moveProxy( m_proxy, m_aabb, disp);
	}
}
