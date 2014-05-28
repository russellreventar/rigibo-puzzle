package physics.dataStorage;

import java.util.HashMap;



import physics.collision.AABB;
import physics.collision.Collision;
import physics.collision.Distance;
import physics.collision.TimeOfImpact;
import physics.dynamics.contacts.CircleContact;
import physics.dynamics.contacts.Contact;
import physics.dynamics.contacts.PolygonAndCircleContact;
import physics.dynamics.contacts.PolygonContact;
import physics.tools.Mat22;
import physics.tools.Settings;
import physics.tools.Vec;

/**
 * Provides object pooling for all objects used in the engine. Objects retrieved from here should
 * only be used temporarily, and then pushed back (with the exception of arrays).
 * 
 */
public class DefaultWorldPool implements IWorldPool {

  private final OrderedStack<Vec> vecs;
  private final OrderedStack<Mat22> mats;
  private final OrderedStack<AABB> aabbs;

  private final HashMap<Integer, float[]> afloats = new HashMap<Integer, float[]>();
  private final HashMap<Integer, int[]> aints = new HashMap<Integer, int[]>();
  private final HashMap<Integer, Vec[]> avecs = new HashMap<Integer, Vec[]>();

  private final Class<?>[] classes = new Class<?>[] { IWorldPool.class };
  private final Object[] args = new Object[] { this };

  private final MutableStack<Contact, PolygonContact> pcstack = new MutableStack<Contact, PolygonContact>(
      PolygonContact.class, Settings.CONTACT_STACK_INIT_SIZE, classes, args);

  private final MutableStack<Contact, CircleContact> ccstack = new MutableStack<Contact, CircleContact>(
      CircleContact.class, Settings.CONTACT_STACK_INIT_SIZE, classes, args);

  private final MutableStack<Contact, PolygonAndCircleContact> cpstack = new MutableStack<Contact, PolygonAndCircleContact>(
      PolygonAndCircleContact.class, Settings.CONTACT_STACK_INIT_SIZE, classes, args);

  private final Collision collision;
  private final TimeOfImpact toi;
  private final Distance dist;

  public DefaultWorldPool(int argSize, int argContainerSize) {
    vecs = new OrderedStack<Vec>(Vec.class, argSize, argContainerSize);
    mats = new OrderedStack<Mat22>(Mat22.class, argSize, argContainerSize);
    aabbs = new OrderedStack<AABB>(AABB.class, argSize, argContainerSize);

    dist = new Distance();
    collision = new Collision(this);
    toi = new TimeOfImpact(this);
  }

  public final IDynamicStack<Contact> getPolyContactStack() {
    return pcstack;
  }

  public final IDynamicStack<Contact> getCircleContactStack() {
    return ccstack;
  }

  public final IDynamicStack<Contact> getPolyCircleContactStack() {
    return cpstack;
  }

  public final Vec popVec2() {
    return vecs.pop();
  }

  public final Vec[] popVec2(int argNum) {
    return vecs.pop(argNum);
  }
  
  public final void pushVec2(int argNum) {
    vecs.push(argNum);
  }

  public final Mat22 popMat22() {
    return mats.pop();
  }

  public final Mat22[] popMat22(int argNum) {
    return mats.pop(argNum);
  }

  public final void pushMat22(int argNum) {
    mats.push(argNum);
  }

  public final AABB popAABB() {
    return aabbs.pop();
  }

  public final AABB[] popAABB(int argNum) {
    return aabbs.pop(argNum);
  }

  public final void pushAABB(int argNum) {
    aabbs.push(argNum);
  }

  public final Collision getCollision() {
    return collision;
  }

  public final TimeOfImpact getTimeOfImpact() {
    return toi;
  }

  public final Distance getDistance() {
    return dist;
  }

  public final float[] getFloatArray(int argLength) {
    if (!afloats.containsKey(argLength)) {
      afloats.put(argLength, new float[argLength]);
    }

    assert (afloats.get(argLength).length == argLength) : "Array not built with correct length";
    return afloats.get(argLength);
  }

  public final int[] getIntArray(int argLength) {
    if (!aints.containsKey(argLength)) {
      aints.put(argLength, new int[argLength]);
    }

    assert (aints.get(argLength).length == argLength) : "Array not built with correct length";
    return aints.get(argLength);
  }

  public final Vec[] getVec2Array(int argLength) {
    if (!avecs.containsKey(argLength)) {
      Vec[] ray = new Vec[argLength];
      for (int i = 0; i < argLength; i++) {
        ray[i] = new Vec();
      }
      avecs.put(argLength, ray);
    }

    assert (avecs.get(argLength).length == argLength) : "Array not built with correct length";
    return avecs.get(argLength);
  }

@Override
public void pushVec3(int argNum) {
	// TODO Auto-generated method stub
	
}
}
