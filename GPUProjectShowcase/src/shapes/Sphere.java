package shapes;

import java.awt.Graphics;

import util.MathTools;
import util.Point;
import util.Point3D;
import util.Vector3D;

public class Sphere extends Shape3D {
	
	public double x, y, z, radius;
	public Point3D centerPoint;
	
	public Sphere(Point3D center, double radius) {
		this.x = center.x;
		this.y = center.y;
		this.z = center.z;
		this.radius = radius;
		this.centerPoint = new Point3D(x, y, z);
	}
	
	public Sphere(int x, int y, int z, int radius) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.radius = radius;
		this.centerPoint = new Point3D(x, y, z);
	}

	public void tick() {}
	public void draw(Graphics g) {}
	
	public util.Vector3D getNormal(Point3D p, Vector3D v) {
		return new Vector3D(p, this.centerPoint);
	}
	
	public util.Point3D getIntersectPoint(Point3D a, Vector3D v) {
		Point3D closePoint = this.getClosePoint(a, v);
		//System.out.println("YES");
		if(closePoint != null && MathTools.dist3D(centerPoint, closePoint) < this.radius) {
			double dist = MathTools.dist3D(closePoint, this.centerPoint);
			double y = Math.sin(Math.acos(dist / this.radius)) * this.radius;
			double distToIntersect = MathTools.dist3D(closePoint, a);
			distToIntersect -= y;
			Point3D nextPos = new Point3D(a);
			Vector3D toNext = new Vector3D(v);
			toNext.setMagnitude(distToIntersect);
			nextPos.addVector(toNext);
			//System.out.println(distToIntersect);
			return nextPos;
		}
		else {
			return null;
		}
	}

	public util.Point3D getClosePoint(Point3D a, Vector3D v) {	//return null if the close point is before the ray starts
		double distToCenter = MathTools.dist3D(a, centerPoint);
		//System.out.println(distToCenter);
		Vector3D add = new Vector3D(v);
		add.setMagnitude(distToCenter + 10);
		Point3D b = new Point3D(a);
		b.addVector(add);
		double dotA = MathTools.dotProduct3D(new Vector3D(a, centerPoint), new Vector3D(a, b));
		double dotB = MathTools.dotProduct3D(new Vector3D(a, b), new Vector3D(a, b));
		Vector3D line = new Vector3D(a, b);
		line.multiply(dotA / dotB);
		Point3D projection = new Point3D(a);
		projection.addVector(line);
		if(MathTools.dotProduct3D(line, v) < 0) {
			return null;
		}
		//System.out.println("YES");
		return projection;
	}

}
