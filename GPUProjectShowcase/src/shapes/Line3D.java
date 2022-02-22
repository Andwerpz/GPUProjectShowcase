package shapes;

import java.awt.Graphics;

import util.Point;
import util.Point3D;
import util.Vector3D;

public class Line3D extends Shape3D{
	
	double x1, y1, z1, x2, y2, z2;
	
	
	public Line3D(Point3D a, Point3D b) {
		x1 = a.x;
		y1 = a.y;
		z1 = a.z;
		x2 = b.x;
		y2 = b.y;
		z2 = b.z;
	}

	public void tick() {}
	public void draw(Graphics g) {}
	
	//projects a 3d point onto this line and returns the projected point

	public util.Point3D getClosePoint(Point3D p, Vector3D v) {
		double length = new Vector3D(x1 - x2, y1 - y2, z1 - z2).getMagnitude();
		return null;
	}

	@Override
	public Point3D getIntersectPoint(Point3D p, Vector3D v) {
		// TODO Auto-generated method stub
		return null;
	}

	public Vector3D getNormal(Point3D p, Vector3D v) {
		// TODO Auto-generated method stub
		return null;
	}

}
