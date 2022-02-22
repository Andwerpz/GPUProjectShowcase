package shapes;

import java.awt.Graphics;

import util.MathTools;
import util.Point;
import util.Vector;

public class Circle extends Shape{
	
	public double x, y, radius;
	
	public Circle(double x, double y, double radius) {
		this.x = x;
		this.y = y;
		this.radius = radius;
	}

	@Override
	public void tick() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void draw(Graphics g) {
		g.drawOval((int) (this.x - this.radius), (int) (this.y - this.radius), (int)(this.radius * 2), (int) (this.radius * 2));
	}

	//distance between a point and the center of the circle
	
	public double getDist(util.Point p) {
		return MathTools.dist(p.x, p.y, this.x, this.y) - (double)(this.radius); 
	}
	
	//gets intersection point between ray and circle in O(1)
	
	public Point getIntersectionPoint(util.Point p, util.Vector v) {
		util.Point center = new util.Point(this.x, this.y);
		double dist = MathTools.dist(p.x, p.y, center.x, center.y);
		util.Point end = new util.Point(p);
		Vector add = new Vector(v);
		add.setMagnitude(dist + 10);
		end.addVector(add);
		Line line = new Line(p, end);
		double curDist = line.getDist(center);
		if(curDist < this.radius) {
			//System.out.println(curDist);
			util.Point closePoint = line.getClosePoint(center);
			if(closePoint == null) {
				return null;
			}
			//g.drawRect((int) closePoint.x, (int) closePoint.y, 5, 5);
			Vector d = new Vector(p, closePoint);
			double actualDist = d.getMagnitude();
			double y = Math.sin(Math.acos(curDist / this.radius)) * this.radius;
			double finalDist = actualDist - y;
			add.setMagnitude(finalDist);
			util.Point ans = new Point(p);
			ans.addVector(add);
			return ans;
		}
		return null;
	}

	//gets the reflection of a vector bouncing off of the circles "surface"
	
	public Vector getReflection(Vector vec, Point p) {
		
		Point intersectPoint = this.getIntersectionPoint(p, vec);
		Point center = new Point(this.x, this.y);
		Vector curDir = new Vector(vec);
		
		Vector normal = new Vector(intersectPoint, center);
		normal.normalize();
		Vector tangent = new Vector(-normal.y, normal.x);
		
		curDir.normalize();
		
		double normalComponent = normal.x * curDir.x + normal.y * curDir.y;
		double tangentComponent = tangent.x * curDir.x + tangent.y * curDir.y;
		
		normal.setMagnitude(-normalComponent);
		tangent.setMagnitude(tangentComponent);
		
		curDir = new Vector(normal);
		curDir.addVector(tangent);
		
		return curDir;
	}

}
