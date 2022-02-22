package shapes;

import java.awt.Graphics;
import java.awt.Point;

import util.Vector;
import util.MathTools;

public class Line extends Shape{
	
	public double x1, y1, x2, y2;
	
	public Line(double x1, double y1, double x2, double y2) {
		this.x1 = x1;
		this.y1 = y1;
		this.x2 = x2;
		this.y2 = y2;
	}
	
	public Line(util.Point a, util.Point b) {
		this.x1 = a.x;
		this.y1 = a.y;
		this.x2 = b.x;
		this.y2 = b.y;
	}
	
	public Line(Point a, Point b) {
		this.x1 = a.x;
		this.y1 = a.y;
		this.x2 = b.x;
		this.y2 = b.y;
	}
	
	public void draw(Graphics g) {
		g.drawLine((int) x1, (int) y1, (int) x2, (int) y2);
	}
	
	//gets the distance between a point and the closest point on the line
	//pretty much just projects the point onto the line, and then returns the distance between the projection and the original point
	
	public double getDist(util.Point p) {
		double dist = Math.min(MathTools.dist(x1, y1, p.x, p.y), MathTools.dist(x2, y2, p.x, p.y));
		double length = MathTools.dist(x1, y1, x2, y2);
		double dot = (((p.x - x1) * (x2 - x1)) + ((p.y - y1) * (y2 - y1))) / Math.pow(length, 2);
		double closeX = x1 + (dot * (x2 - x1));
		double closeY = y1 + (dot * (y2 - y1));
		if(isOnLine(closeX, closeY)) {
			dist = Math.min(dist, MathTools.dist(p.x, p.y, closeX, closeY));
		}
		return dist;
	}
	
	//same as this.getDist() but now it returns the point on the line in question
	
	public util.Point getClosePoint(util.Point p){
		double dist = Math.min(MathTools.dist(x1, y1, p.x, p.y), MathTools.dist(x2, y2, p.x, p.y));
		double length = MathTools.dist(x1, y1, x2, y2);
		double dot = (((p.x - x1) * (x2 - x1)) + ((p.y - y1) * (y2 - y1))) / Math.pow(length, 2);
		double closeX = x1 + (dot * (x2 - x1));
		double closeY = y1 + (dot * (y2 - y1));
		if(isOnLine(closeX, closeY)) {
			return new util.Point(closeX, closeY);
		}
		else {
			return null;
		}
	}
	
	//returns true if the point (x, y) is on the line
	
	public boolean isOnLine(double x, double y) {
		double buffer = 0.001;
		double length = MathTools.dist(x1, y1, x2, y2);
		double len1 = MathTools.dist(x, y, x1, y1);
		double len2 = MathTools.dist(x, y, x2, y2);
		if(Math.abs((len1 + len2) - length) < buffer) {
			return true;
		}
		return false;
	}
	
	//TODO this is completely broken
	
	public Vector getReflection(Vector vec, util.Point p) {
		//TODO this is completely broken
		Vector v = new Vector(vec);
		Vector line = new Vector(x2 - x1, y2 - y1);
		double angle = line.getRotationRadians();
		double length = MathTools.dist(x1, y1, x2, y2);
		double dot = (((p.x - x1) * (x2 - x1)) + ((p.y - y1) * (y2 - y1))) / Math.pow(length, 2);
		double closeX = x1 + (dot * (x2 - x1));
		double closeY = y1 + (dot * (y2 - y1));
		Vector normal = new Vector(p.x - closeX, p.y - closeY);
		Vector normalReverse = new Vector(closeX - p.x, closeY - p.y);
		Line direction = new Line((int) p.x, (int) p.y, (int) (p.x + v.x), (int) (p.y + v.y));
		util.Point intersection = this.lineIntersection(direction);
		Vector d = new Vector(intersection.x - p.x, intersection.y - p.y);
		Vector tangent = new Vector(d.x - (normalReverse.x), d.y - (normalReverse.y));
		util.Point newD = new util.Point(intersection.x, intersection.y);
		newD.addVector(tangent);
		newD.addVector(normal);
		return new Vector(newD.x - intersection.x, newD.y - intersection.y);
	}
	
	//returns the point at which another line intersects this line.
	
	//can be used as a substitute for ray / line intersection if the input line is made to be very long
	
	public util.Point lineIntersection(Line a) {
		double x3 = a.x1;
		double x4 = a.x2;
		double y3 = a.y1;
		double y4 = a.y2;
		
		double uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
		double uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
		
		double intersectionX = x1 + (uA * (x2-x1));
		double intersectionY = y1 + (uA * (y2-y1));
		
		if(this.isOnLine(intersectionX, intersectionY) && a.isOnLine(intersectionX, intersectionY)) {
			return new util.Point(intersectionX, intersectionY);
		}
		
		return null;
	}

	@Override
	public void tick() {
		// TODO Auto-generated method stub
		
	}

}
