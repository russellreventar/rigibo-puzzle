package physics.collision;

import physics.tools.Mat22;
import physics.tools.MathUtils;
import physics.tools.Settings;
import physics.tools.Transform;
import physics.tools.Vec;

// updated to rev 100
/**
 * This is used to compute the current state of a contact manifold.
 * 
 */
public class WorldManifold {
	/**
	 * World vector pointing from A to B
	 */
	public final Vec normal;
	
	/**
	 * World contact point (point of intersection)
	 */
	public final Vec[] points;
	
	public WorldManifold() {
		normal = new Vec();
		points = new Vec[Settings.maxManifoldPoints];
		for (int i = 0; i < Settings.maxManifoldPoints; i++) {
			points[i] = new Vec();
		}
	}

	private final Vec pool3 = new Vec();
	private final Vec pool4 = new Vec();
	
	public final void initialize(final Manifold manifold, final Transform xfA, float radiusA, final Transform xfB,
			float radiusB) {
		if (manifold.pointCount == 0) {
			return;
		}
		
		switch (manifold.type) {
			case CIRCLES :{
//				final Vec2 pointA = pool3;
//				final Vec2 pointB = pool4;
//				
//				normal.set(1, 0);
//				Transform.mulToOut(xfA, manifold.localPoint, pointA);
//				Transform.mulToOut(xfB, manifold.points[0].localPoint, pointB);
//				
//				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
//					normal.set(pointB).subLocal(pointA);
//					normal.normalize();
//				}
//				
//				cA.set(normal).mulLocal(radiusA).addLocal(pointA);
//				cB.set(normal).mulLocal(radiusB).subLocal(pointB).negateLocal();
//				points[0].set(cA).addLocal(cB).mulLocal(0.5f);
				final Vec pointA = pool3;
				final Vec pointB = pool4;
				
				normal.x = 1;
				normal.y = 0;
				pointA.x = xfA.position.x + xfA.R.col1.x * manifold.localPoint.x + xfA.R.col2.x * manifold.localPoint.y;
				pointA.y = xfA.position.y + xfA.R.col1.y * manifold.localPoint.x + xfA.R.col2.y * manifold.localPoint.y;
				pointB.x = xfB.position.x + xfB.R.col1.x * manifold.points[0].localPoint.x + xfB.R.col2.x * manifold.points[0].localPoint.y;
				pointB.y = xfB.position.y + xfB.R.col1.y * manifold.points[0].localPoint.x + xfB.R.col2.y * manifold.points[0].localPoint.y;
				
				if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
					normal.x = pointB.x - pointA.x;
					normal.y = pointB.y - pointA.y;
					normal.normalize();
				}
				
				final float cAx = normal.x * radiusA + pointA.x;
				final float cAy = normal.y * radiusA + pointA.y;
				
				final float cBx = -normal.x * radiusB + pointB.x;
				final float cBy = -normal.y * radiusB + pointB.y;

				points[0].x = (cAx + cBx) *.5f;
				points[0].y = (cAy + cBy) *.5f;
			}
				break;
			case FACE_A : {
//				final Vec2 planePoint = pool3;
//				
//				Mat22.mulToOut(xfA.R, manifold.localNormal, normal);
//				Transform.mulToOut(xfA, manifold.localPoint, planePoint);
//				
//				final Vec2 clipPoint = pool4;
//				
//				for (int i = 0; i < manifold.pointCount; i++) {
//					// b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
//					// b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint,
//					// normal)) * normal;
//					// b2Vec2 cB = clipPoint - radiusB * normal;
//					// points[i] = 0.5f * (cA + cB);
//					Transform.mulToOut(xfB, manifold.points[i].localPoint, clipPoint);
//					// use cA as temporary for now
//					cA.set(clipPoint).subLocal(planePoint);
//					float scalar = radiusA - Vec2.dot(cA, normal);
//					cA.set(normal).mulLocal(scalar).addLocal(clipPoint);
//					cB.set(normal).mulLocal(radiusB).subLocal(clipPoint).negateLocal();
//					points[i].set(cA).addLocal(cB).mulLocal(0.5f);
//				}
				final Vec planePoint = pool3;
				
				normal.x = xfA.R.col1.x * manifold.localNormal.x + xfA.R.col2.x * manifold.localNormal.y;
				normal.y = xfA.R.col1.y * manifold.localNormal.x + xfA.R.col2.y * manifold.localNormal.y;
				planePoint.x = xfA.position.x + xfA.R.col1.x * manifold.localPoint.x + xfA.R.col2.x * manifold.localPoint.y;
				planePoint.y = xfA.position.y + xfA.R.col1.y * manifold.localPoint.x + xfA.R.col2.y * manifold.localPoint.y;
				
				final Vec clipPoint = pool4;
				
				for (int i = 0; i < manifold.pointCount; i++) {
					// b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
					// b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint,
					// normal)) * normal;
					// b2Vec2 cB = clipPoint - radiusB * normal;
					// points[i] = 0.5f * (cA + cB);
					

					clipPoint.x = xfB.position.x + xfB.R.col1.x * manifold.points[i].localPoint.x + xfB.R.col2.x * manifold.points[i].localPoint.y;
					clipPoint.y = xfB.position.y + xfB.R.col1.y * manifold.points[i].localPoint.x + xfB.R.col2.y * manifold.points[i].localPoint.y;
					
					final float scalar = radiusA - ((clipPoint.x - planePoint.x) * normal.x + (clipPoint.y - planePoint.y) * normal.y);
					
					final float cAx = normal.x * scalar + clipPoint.x;
					final float cAy = normal.y * scalar + clipPoint.y;
					
					final float cBx = - normal.x * radiusB + clipPoint.x;
					final float cBy = - normal.y * radiusB + clipPoint.y;
					
					points[i].x = (cAx + cBx)*.5f;
					points[i].y = (cAy + cBy)*.5f;
				}
			}
				break;
			case FACE_B :
				final Vec planePoint = pool3;
				
				final Mat22 R = xfB.R;
				normal.x = R.col1.x * manifold.localNormal.x + R.col2.x * manifold.localNormal.y;
				normal.y = R.col1.y * manifold.localNormal.x + R.col2.y * manifold.localNormal.y;
				final Vec v = manifold.localPoint;
				planePoint.x = xfB.position.x + xfB.R.col1.x * v.x + xfB.R.col2.x * v.y;
				planePoint.y = xfB.position.y + xfB.R.col1.y * v.x + xfB.R.col2.y * v.y;
				
				final Vec clipPoint = pool4;
				
					
                for (int i = 0; i < manifold.pointCount; i++) {
					clipPoint.x = xfA.position.x + xfA.R.col1.x * manifold.points[i].localPoint.x + xfA.R.col2.x * manifold.points[i].localPoint.y;
					clipPoint.y = xfA.position.y + xfA.R.col1.y * manifold.points[i].localPoint.x + xfA.R.col2.y * manifold.points[i].localPoint.y;
					
					final float scalar = radiusB - ((clipPoint.x - planePoint.x) * normal.x + (clipPoint.y - planePoint.y) * normal.y);
					
					final float cBx =  normal.x * scalar + clipPoint.x;
					final float cBy =  normal.y * scalar + clipPoint.y;
					
					final float cAx = - normal.x * radiusA + clipPoint.x;
					final float cAy = - normal.y * radiusA + clipPoint.y;
					
					points[i].x = (cAx + cBx) *.5f;
					points[i].y = (cAy + cBy) *.5f;
				}
				// Ensure normal points from A to B.
				normal.x = -normal.x;
				normal.y = -normal.y;
				break;
		}
	}
}
