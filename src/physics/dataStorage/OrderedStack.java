package physics.dataStorage;

import java.lang.reflect.Array;
/**
 */
public class OrderedStack<E> {
  //private static final Logger log = LoggerFactory.getLogger(OrderedStack.class);

  private final E[] pool;
  private int index;
  private final int size;
  private final E[] container;

  @SuppressWarnings("unchecked")
  public OrderedStack(Class<E> argClass, int argStackSize, int argContainerSize) {
    size = argStackSize;
    pool = (E[]) Array.newInstance(argClass, argStackSize);
    for (int i = 0; i < argStackSize; i++) {
      try {
        pool[i] = argClass.newInstance();
      } catch (InstantiationException e) {
       // log.error("Error creating pooled object " + argClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + argClass.getCanonicalName();
      } catch (IllegalAccessException e) {
       // log.error("Error creating pooled object " + argClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + argClass.getCanonicalName();
      }
    }
    index = 0;
    container = (E[]) Array.newInstance(argClass, argContainerSize);
  }

  public final E pop() {
    assert (index < size) : "End of stack reached, there is probably a leak somewhere";
    return pool[index++];
  }

  public final E[] pop(int argNum) {
    assert (index + argNum < size) : "End of stack reached, there is probably a leak somewhere";
    assert (argNum <= container.length) : "Container array is too small";
    System.arraycopy(pool, index, container, 0, argNum);
    index += argNum;
    return container;
  }

  public final void push(int argNum) {
    index -= argNum;
    assert (index >= 0) : "Beginning of stack reached, push/pops are unmatched";
  }
}
