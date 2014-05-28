package physics.dynamics.contacts;


import physics.collision.Manifold;
import physics.collision.shapes.Polygon;
import physics.collision.shapes.ShapeType;
import physics.dataStorage.IWorldPool;
import physics.dynamics.Fixture;
import physics.tools.Transform;

public class PolygonContact extends Contact {
	
	public PolygonContact(IWorldPool argPool) {
		super(argPool);
	}

	public void init(Fixture fixtureA, Fixture fixtureB) {
		super.init(fixtureA, fixtureB);
		assert(m_fixtureA.getType() == ShapeType.POLYGON);
		assert(m_fixtureB.getType() == ShapeType.POLYGON);
	}

	@Override
	public void evaluate(Manifold manifold, Transform xfA, Transform xfB) {
		pool.getCollision().collidePolygons(m_manifold,
				(Polygon)m_fixtureA.getShape(), xfA,
				(Polygon)m_fixtureB.getShape(), xfB);
	}
}
