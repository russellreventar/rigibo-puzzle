package physics.collision.shapes;

public enum ShapeType {
	UNKNOWN(-1), CIRCLE(0), POLYGON(1);
	public static final int TYPE_COUNT = 2;
	
	public final int intValue;
	
	ShapeType(int argValue){
		intValue = argValue;
	}
}
