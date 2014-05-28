package physics.tools;

import java.io.Serializable;

/**
 * A 2-by-2 matrix. Stored in column-major order.
 */
public class Mat22 implements Serializable{
	private static final long serialVersionUID = 1L;
	
	public final Vec col1, col2;

	/** Convert the matrix to printable format. */
	@Override
	public String toString() {
		String s = "";
		s += "["+col1.x+","+col2.x+"]\n";
		s += "["+col1.y+","+col2.y+"]";
		return s;
	}

	/** Construct zero matrix.  Note: this is NOT an identity matrix!
	 * djm fixed double allocation problem*/
	public Mat22() {
		col1 = new Vec();
		col2 = new Vec();
	}

	/**
	 * Create a matrix with given vectors as columns.
	 * @param c1 Column 1 of matrix
	 * @param c2 Column 2 of matrix
	 */
	public Mat22(final Vec c1, final Vec c2) {
		col1 = c1.clone();
		col2 = c2.clone();
	}

	/**
	 * Create a matrix from four floats.
	 * @param col1x
	 * @param col2x
	 * @param col1y
	 * @param col2y
	 */
	public Mat22(final float col1x, final float col2x, final float col1y, final float col2y) {
		col1 = new Vec(col1x, col1y);
		col2 = new Vec(col2x, col2y);
	}

	/**
	 * Set as a copy of another matrix.
	 * @param m Matrix to copy
	 */
	public final Mat22 set(final Mat22 m) {
		col1.x = m.col1.x;
		col1.y = m.col1.y;
		col2.x = m.col2.x;
		col2.y = m.col2.y;
		return this;
	}

	public final Mat22 set(final float col1x, final float col2x, final float col1y, final float col2y) {
		col1.x = col1x;
		col1.y = col1y;
		col2.x = col2x;
		col2.y = col2y;
		return this;
	}

	/**
	 * Return a clone of this matrix.
	 * djm fixed double allocation
	 */
	@Override
	public final Mat22 clone() {
		return new Mat22(col1, col2);
	}

	/**
	 * Set as a matrix representing a rotation.
	 * @param angle Rotation (in radians) that matrix represents.
	 */
	public final void set(final float angle) {
		final float c = MathUtils.cos(angle), s = MathUtils.sin(angle);
		col1.x = c; col2.x = -s;
		col1.y = s; col2.y = c;
	}

	/**
	 * Set as the identity matrix.
	 */
	public final void setIdentity() {
		col1.x = 1.0f;
		col2.x = 0.0f;
		col1.y = 0.0f;
		col2.y = 1.0f;
	}

	/**
	 * Set as the zero matrix.
	 */
	public final void setZero() {
		col1.x = 0.0f;
		col2.x = 0.0f;
		col1.y = 0.0f;
		col2.y = 0.0f;
	}
	
	/**
	 * Extract the angle from this matrix (assumed to be
	 * a rotation matrix).
	 * @return
	 */
	public final float getAngle(){
		return MathUtils.atan2(col1.y, col1.x);
	}

	/**
	 * Set by column vectors.
	 * @param c1 Column 1
	 * @param c2 Column 2
	 */
	public final void set(final Vec c1, final Vec c2) {
		col1.x = c1.x;
		col2.x = c2.x;
		col1.y = c1.y;
		col2.y = c2.y;
	}

	/** Returns the inverted Mat22 - does NOT invert the matrix locally! */
	public final Mat22 invert() {
		final float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		final Mat22 B = new Mat22();
		float det = a * d - b * c;
		if(det != 0){
			det = 1.0f / det;
		}
		B.col1.x = det * d;
		B.col2.x = -det * b;
		B.col1.y = -det * c;
		B.col2.y = det * a;
		return B;
	}
	
	public final Mat22 invertLocal() {
		final float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		float det = a * d - b * c;
		if(det != 0){
			det = 1.0f / det;
		}
		col1.x = det * d;
		col2.x = -det * b;
		col1.y = -det * c;
		col2.y = det * a;
		return this;
	}

	public final void invertToOut(final Mat22 out){
		final float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		float det = a * d - b * c;
		// b2Assert(det != 0.0f);
		det = 1.0f / det;
		out.col1.x = det * d;
		out.col2.x = -det * b;
		out.col1.y = -det * c;
		out.col2.y = det * a;
	}

	/**
	 * Return the matrix composed of the absolute values of all elements.
	 * djm: fixed double allocation
	 * @return Absolute value matrix
	 */
	public final Mat22 abs() {
		return new Mat22(MathUtils.abs(col1.x),
		                 MathUtils.abs(col2.x),
		                 MathUtils.abs(col1.y),
		                 MathUtils.abs(col2.y));
	}

	/* djm: added */
	public final void absLocal(){
		col1.absLocal();
		col2.absLocal();
	}

	/**
	 * Return the matrix composed of the absolute values of all elements.
	 * @return Absolute value matrix
	 */
	public final static Mat22 abs(final Mat22 R) {
		return R.abs();
	}

	/* djm created */
	public static void absToOut(final Mat22 R, final Mat22 out){
		out.col1.x = MathUtils.abs(R.col1.x);
		out.col1.y = MathUtils.abs(R.col1.y);
		out.col2.x = MathUtils.abs(R.col2.x);
		out.col2.y = MathUtils.abs(R.col2.y);
	}

	/**
	 * Multiply a vector by this matrix.
	 * @param v Vector to multiply by matrix.
	 * @return Resulting vector
	 */
	public final Vec mul(final Vec v) {
		return new Vec(col1.x * v.x + col2.x * v.y, col1.y * v.x + col2.y
		                * v.y);
	}
	
	/* djm added */
	public final void mulToOut(final Vec v, final Vec out){
		final float tempy = col1.y * v.x + col2.y * v.y;
		out.x = col1.x * v.x + col2.x * v.y;
		out.y = tempy;
	}


	/**
	 * Multiply another matrix by this one (this one on left).
	 * djm optimized
	 * @param R
	 * @return
	 */
	public final Mat22 mul(final Mat22 R) {
		/*Mat22 C = new Mat22();
		 *C.set(this.mul(R.col1), this.mul(R.col2));
		 *return C;
		 */
		final Mat22 C = new Mat22();
		C.col1.x = col1.x * R.col1.x + col2.x * R.col1.y;
		C.col1.y = col1.y * R.col1.x + col2.y * R.col1.y;
		C.col2.x = col1.x * R.col2.x + col2.x * R.col2.y;
		C.col2.y = col1.y * R.col2.x + col2.y * R.col2.y;
		//C.set(col1,col2);
		return C;
	}
	
	public final Mat22 mulLocal(final Mat22 R){
		mulToOut(R, this);
		return this;
	}

	/* djm: created */
	public final void mulToOut(final Mat22 R, final Mat22 out){
		final float tempy1 = this.col1.y * R.col1.x + this.col2.y * R.col1.y;
		final float tempx1 = this.col1.x * R.col1.x + this.col2.x * R.col1.y;
		out.col1.x = tempx1;
		out.col1.y = tempy1;
		final float tempy2 = this.col1.y * R.col2.x + this.col2.y * R.col2.y;
		final float tempx2 = this.col1.x * R.col2.x + this.col2.x * R.col2.y;
		out.col2.x = tempx2;
		out.col2.y = tempy2;
	}

	/**
	 * Multiply another matrix by the transpose of this one (transpose of this one on left).
	 * djm: optimized
	 * @param B
	 * @return
	 */
	public final Mat22 mulTrans(final Mat22 B) {
		/*
		 * Vec2 c1 = new Vec2(Vec2.dot(this.col1, B.col1), Vec2.dot(this.col2,
                B.col1));
        Vec2 c2 = new Vec2(Vec2.dot(this.col1, B.col2), Vec2.dot(this.col2,
                B.col2));
        Mat22 C = new Mat22();
        C.set(c1, c2);
        return C;
		 */
		final Mat22 C = new Mat22();

		C.col1.x = Vec.dot(this.col1, B.col1);
		C.col1.y = Vec.dot(this.col2, B.col1);

		C.col2.x = Vec.dot(this.col1, B.col2);
		C.col2.y = Vec.dot(this.col2, B.col2);
		return C;
	}
	
	public final Mat22 mulTransLocal(final Mat22 B){
		mulTransToOut(B, this);
		return this;
	}

	/* djm added */
	public final void mulTransToOut(final Mat22 B, final Mat22 out){
		/*out.col1.x = Vec2.dot(this.col1, B.col1);
		out.col1.y = Vec2.dot(this.col2, B.col1);
		out.col2.x = Vec2.dot(this.col1, B.col2);
		out.col2.y = Vec2.dot(this.col2, B.col2);*/
		final float x1 = this.col1.x * B.col1.x + this.col1.y * B.col1.y;
		final float y1 = this.col2.x * B.col1.x + this.col2.y * B.col1.y;
		final float x2 = this.col1.x * B.col2.x + this.col1.y * B.col2.y;
		final float y2 = this.col2.x * B.col2.x + this.col2.y * B.col2.y;
		out.col1.x = x1;
		out.col2.x = x2;
		out.col1.y = y1;
		out.col2.y = y2;
	}

	/**
	 * Multiply a vector by the transpose of this matrix.
	 * @param v
	 * @return
	 */
	public final Vec mulTrans(final Vec v) {
		//return new Vec2(Vec2.dot(v, col1), Vec2.dot(v, col2));
		return new Vec((v.x * col1.x + v.y * col1.y), (v.x * col2.x + v.y * col2.y));
	}

	/* djm added */
	public final void mulTransToOut(final Vec v, final Vec out){
		/*out.x = Vec2.dot(v, col1);
		out.y = Vec2.dot(v, col2);*/
		out.x = v.x * col1.x + v.y * col1.y;
		out.y = v.x * col2.x + v.y * col2.y;
	}

	/**
	 * Add this matrix to B, return the result.
	 * @param B
	 * @return
	 */
	public final Mat22 add(final Mat22 B) {
		//return new Mat22(col1.add(B.col1), col2.add(B.col2));	
		Mat22 m = new Mat22();
		m.col1.x = col1.x + B.col1.x;
		m.col1.y = col1.y + B.col1.y;
		m.col2.x = col2.x + B.col2.x;
		m.col2.y = col2.y + B.col2.y;
		return m;
	}

	/**
	 * Add B to this matrix locally.
	 * @param B
	 * @return
	 */
	public final Mat22 addLocal(final Mat22 B) {
		//col1.addLocal(B.col1);
		//col2.addLocal(B.col2);
		col1.x += B.col1.x;
		col1.y += B.col1.y;
		col2.x += B.col2.x;
		col2.y += B.col2.y;
		return this;
	}

	/**
	 * Solve A * x = b where A = this matrix.
	 * @return The vector x that solves the above equation.
	 */
	public final Vec solve(final Vec b) {
		final float a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		float det = a11 * a22 - a12 * a21;
		if (det != 0.0f){
			det = 1.0f / det;
		}
		final Vec x = new Vec( det * (a22 * b.x - a12 * b.y),
		                         det * (a11 * b.y - a21 * b.x) );
		return x;
	}

	/* djm added */
	public final void solveToOut(final Vec b, final Vec out) {
		final float a11 = col1.x, a12 = col2.x, a21 = col1.y, a22 = col2.y;
		float det = a11 * a22 - a12 * a21;
		if (det != 0.0f){
			det = 1.0f / det;
		}
		final float tempy =  det * (a11 * b.y - a21 * b.x) ;
		out.x = det * (a22 * b.x - a12 * b.y);
		out.y = tempy;
	}

	public final static Vec mul(final Mat22 R, final Vec v) {
		// return R.mul(v);
		return new Vec(R.col1.x * v.x + R.col2.x * v.y, R.col1.y * v.x + R.col2.y
		                * v.y);
	}

	/* djm added */
	public final static void mulToOut(final Mat22 R, final Vec v, final Vec out) {
		// R.mulToOut(v,out);
		final float tempy = R.col1.y * v.x + R.col2.y * v.y;
		out.x = R.col1.x * v.x + R.col2.x * v.y;
		out.y = tempy;
	}

	public final static Mat22 mul(final Mat22 A, final Mat22 B){
		//return A.mul(B);
		final Mat22 C = new Mat22();
		C.col1.x = A.col1.x * B.col1.x + A.col2.x * B.col1.y;
		C.col1.y = A.col1.y * B.col1.x + A.col2.y * B.col1.y;
		C.col2.x = A.col1.x * B.col2.x + A.col2.x * B.col2.y;
		C.col2.y = A.col1.y * B.col2.x + A.col2.y * B.col2.y;
		return C;
	}

	/* djm added */
	public final static void mulToOut(final Mat22 A, final Mat22 B, final Mat22 out){
		final float tempy1 = A.col1.y * B.col1.x + A.col2.y * B.col1.y;
		final float tempx1 = A.col1.x * B.col1.x + A.col2.x * B.col1.y;
		final float tempy2 = A.col1.y * B.col2.x + A.col2.y * B.col2.y;
		final float tempx2 = A.col1.x * B.col2.x + A.col2.x * B.col2.y;
		out.col1.x = tempx1;
		out.col1.y = tempy1;
		out.col2.x = tempx2;
		out.col2.y = tempy2;
	}

	public final static Vec mulTrans(final Mat22 R, final Vec v) {
		return new Vec((v.x * R.col1.x + v.y * R.col1.y), (v.x * R.col2.x + v.y * R.col2.y));
	}

	/* djm added */
	public final static void mulTransToOut(final Mat22 R, final Vec v, final Vec out) {
		//R.mulTransToOut(v, out);
		float outx = v.x * R.col1.x + v.y * R.col1.y;
		out.y = v.x * R.col2.x + v.y * R.col2.y;
		out.x = outx;
	}

	public final static Mat22 mulTrans(final Mat22 A, final Mat22 B){
		//return A.mulTrans(B);
		final Mat22 C = new Mat22();
		C.col1.x = A.col1.x * B.col1.x + A.col1.y * B.col1.y;
		C.col1.y = A.col2.x * B.col1.x + A.col2.y * B.col1.y;
		C.col2.x = A.col1.x * B.col2.x + A.col1.y * B.col2.y;
		C.col2.y = A.col2.x * B.col2.x + A.col2.y * B.col2.y;
		return C;
	}

	/* djm added */
	public final static void mulTransToOut(final Mat22 A, final Mat22 B, final Mat22 out){
		final float x1 = A.col1.x * B.col1.x + A.col1.y * B.col1.y;
		final float y1 = A.col2.x * B.col1.x + A.col2.y * B.col1.y;
		final float x2 = A.col1.x * B.col2.x + A.col1.y * B.col2.y;
		final float y2 = A.col2.x * B.col2.x + A.col2.y * B.col2.y;
		
		out.col1.x = x1;
		out.col1.y = y1;
		out.col2.x = x2;
		out.col2.y = y2;
	}
	
	public final static Mat22 createRotationalTransform(float angle){
		Mat22 mat = new Mat22();
		final float c = MathUtils.cos(angle);
		final float s = MathUtils.sin(angle);
		mat.col1.x = c;
		mat.col2.x = -s;
		mat.col1.y = s;
		mat.col2.y = c;
		return mat;
	}
	
	public final static void createRotationalTransform(float angle, Mat22 out){
		final float c = MathUtils.cos(angle);
		final float s = MathUtils.sin(angle);
		out.col1.x = c;
		out.col2.x = -s;
		out.col1.y = s;
		out.col2.y = c;
	}
	
	public final static Mat22 createScaleTransform(float scale){
		Mat22 mat = new Mat22();
		mat.col1.x = scale;
		mat.col2.y = scale;
		return mat;
	}
	
	public final static void createScaleTransform(float scale, Mat22 out){
		out.col1.x = scale;
		out.col2.y = scale;
	}
}