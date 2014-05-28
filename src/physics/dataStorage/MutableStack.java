package physics.dataStorage;

import java.lang.reflect.Array;

import java.lang.reflect.InvocationTargetException;


public class MutableStack<E, T extends E> implements IDynamicStack<E> {

  //private static final Logger log = LoggerFactory.getLogger(MutableStack.class);

  private T[] stack;
  private int index;
  private int size;
  private final Class<T> sClass;

  private final Class<?>[] params;
  private final Object[] args;

  public MutableStack(Class<T> argClass, int argInitSize) {
    this(argClass, argInitSize, null, null);
  }

  public MutableStack(Class<T> argClass, int argInitSize, Class<?>[] argParam, Object[] argArgs) {
    index = 0;
    sClass = argClass;
    params = argParam;
    args = argArgs;

    stack = null;
    index = 0;
    extendStack(argInitSize);
  }

  @SuppressWarnings("unchecked")
  private void extendStack(int argSize) {
    T[] newStack = (T[]) Array.newInstance(sClass, argSize);
    if (stack != null) {
      System.arraycopy(stack, 0, newStack, 0, size);
    }
    for (int i = 0; i < newStack.length; i++) {
      try {
        if (params != null) {
          newStack[i] = sClass.getConstructor(params).newInstance(args);
        } else {
          newStack[i] = sClass.newInstance();
        }
      } catch (InstantiationException e) {
      //  log.error("Error creating pooled object " + sClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + sClass.getCanonicalName();
      } catch (IllegalAccessException e) {
      //  log.error("Error creating pooled object " + sClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + sClass.getCanonicalName();
      } catch (IllegalArgumentException e) {
     //   log.error("Error creating pooled object " + sClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + sClass.getCanonicalName();
      } catch (SecurityException e) {
     //   log.error("Error creating pooled object " + sClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + sClass.getCanonicalName();
      } catch (InvocationTargetException e) {
      //  log.error("Error creating pooled object " + sClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + sClass.getCanonicalName();
      } catch (NoSuchMethodException e) {
      //  log.error("Error creating pooled object " + sClass.getSimpleName(), e);
        assert (false) : "Error creating pooled object " + sClass.getCanonicalName();
      }
    }
    stack = newStack;
    size = newStack.length;
  }

  /* (non-Javadoc)
   * 
   * @see org.jbox2d.pooling.IDynamicStack#pop() */
  public final E pop() {
    if (index >= size) {
      extendStack(size * 2);
    }
    return stack[index++];
  }

  /* (non-Javadoc)
   * 
   * @see org.jbox2d.pooling.IDynamicStack#push(E) */
  @SuppressWarnings("unchecked")
  public final void push(E argObject) {
    assert (index > 0);
    stack[--index] = (T) argObject;
  }
}
