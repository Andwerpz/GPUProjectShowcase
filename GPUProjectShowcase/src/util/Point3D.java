package util;

public class Point3D {

	public double x, y, z;
	
	public Point3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public Point3D(Point3D p) {
		this.x = p.x;
		this.y = p.y;
		this.z = p.z;
	}
	
	public void addVector(Vector3D v) {
		this.x += v.x;
		this.y += v.y;
		this.z += v.z;
	}
	
	public void subtractVector(Vector3D v) {
		this.x -= v.x;
		this.y -= v.y;
		this.z -= v.z;
	}
	
	public void rotate(double radX, double radY, double radZ) {
		rotateX(radX);
		rotateY(radY);
		rotateZ(radZ);
	}
	
	public void rotateX(double xRot) {
		double x = this.x;
		double y = this.y;
		double z = this.z;
		this.x = x;
		this.y = ((y * Math.cos(xRot)) + (z * -Math.sin(xRot)));
		this.z = ((y * Math.sin(xRot)) + (z * Math.cos(xRot)));
	}
	public void rotateY(double yRot) {
		double x = this.x;
		double y = this.y;
		double z = this.z;
		this.x = (x * Math.cos(yRot)) + (z * Math.sin(yRot));
		this.y = y;
		this.z = (x * -Math.sin(yRot)) + (z * Math.cos(yRot));
	}
	public void rotateZ(double zRot) {
		double x = this.x;
		double y = this.y;
		double z = this.z;
		this.x = (x * Math.cos(zRot)) + (y * -Math.sin(zRot));
		this.y = (x * Math.sin(zRot)) + (y * Math.cos(zRot));
		this.z = z;
	}
	
}
