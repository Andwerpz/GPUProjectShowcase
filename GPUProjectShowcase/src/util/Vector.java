package util;

public class Vector extends util.Point{

	public Vector(double x, double y) {
		super(x, y);
	}
	
	//pointing from point a to point b
	
	public Vector(util.Point a, util.Point b) {
		super(b.x - a.x, b.y - a.y);
	}
	
	public Vector(Vector a) {	//so you don't have any pointer issues
		super(a.x, a.y);
	}
	
	public Vector(util.Point a) {
		super(a.x, a.y);
	}
	
	public double getMagnitude() {
		return Math.sqrt(this.x * this.x + this.y * this.y);
	}
	
	public void normalize() {
		double mag = this.getMagnitude();
		this.x /= mag;
		this.y /= mag;
	}
	
	public void setMagnitude(double mag) {
		this.normalize();
		this.x *= mag;
		this.y *= mag;
	}
	
	public void rotateCounterClockwise(double rad) {
		double x = this.x;
		double y = this.y;
		this.x = (x * Math.cos(rad)) - (y * Math.sin(rad));
		this.y = (x * Math.sin(rad)) + (y * Math.cos(rad));
	}
	
	public double getRotationRadians() {
		return Math.asin(this.y / this.getMagnitude());
	}
	
	public void multiply(double val) {
		this.x *= val;
		this.y *= val;
	}

}
