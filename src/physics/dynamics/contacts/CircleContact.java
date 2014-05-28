package physics.dynamics.contacts;

import physics.collision.Manifold;
import physics.collision.shapes.Circle;
import physics.collision.shapes.ShapeType;
import physics.dataStorage.IWorldPool;
import physics.dynamics.Fixture;
import physics.tools.Transform;

public class CircleContact extends Contact {
	
	/**
	 * @param argPool
	 */
	public CircleContact(IWorldPool argPool) {
		super(argPool);
	}

	public void init(Fixture fixtureA, Fixture fixtureB){
		super.init(fixtureA, fixtureB);
		assert(m_fixtureA.getType() == ShapeType.CIRCLE);
		assert(m_fixtureB.getType() == ShapeType.CIRCLE);
	}
	
	@Override
	public void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
		pool.getCollision().collideCircles(manifold,
				(Circle)m_fixtureA.getShape(), xfA,
				(Circle)m_fixtureB.getShape(), xfB);
	}
}
