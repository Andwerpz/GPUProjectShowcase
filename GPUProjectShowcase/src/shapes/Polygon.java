package shapes;

import java.awt.Graphics;
import java.util.ArrayList;

import util.MathTools;
import util.Point;
import util.Vector;

public class Polygon extends Shape {
	
	public ArrayList<Point> verticies;
	public ArrayList<Line> lines;
	
	public Polygon(ArrayList<Point> v) {	//verticies must be defined counterclockwise for lines to be drawn properly
		verticies = v;
		lines = new ArrayList<Line>();
		for(int i = 0; i < v.size(); i++) {
			lines.add(new Line(v.get(i), v.get((i + 1) % v.size())));
		}
	}

	@Override
	public void tick() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void draw(Graphics g) {
		for(Line l : lines) {
			l.draw(g);
		}
	}
	
	//returns the closest point on the polygons surface to the input point

	@Override
	public double getDist(util.Point p) {
		double minDist = 0;
		for(int i = 0; i < lines.size(); i++) {
			if(i == 0) {
				minDist = lines.get(i).getDist(p);
			}
			else {
				minDist = Math.min(minDist, lines.get(i).getDist(p));
			}
		}
		return minDist;
	}
	
	//returns the intersection point between this polygon and a ray
	
	//the current solution involves making a new line object of length 1 billion, and checking for intersection that way. As long as big numbers aren't involved with the shapes, 
	//then this solution should work
	
	public util.Point getIntersectionPoint(util.Point p, Vector v){
		double minDist = -1;
		
		Vector next = new Vector(v);
		next.setMagnitude(1000000000);	//1 billion, very bandaid solution
		Point a = new Point(p);
		Point b = new Point(v);
		b.addVector(next);
		
		Point intersectPoint = null;
		
		Line cur = new Line(a, b);
		for(Line l : lines) {
			Point nextIntersect = l.lineIntersection(cur);
			if(nextIntersect != null) {
				double dist = MathTools.dist(nextIntersect.x, nextIntersect.y, a.x, a.y);
				if(minDist == -1) {
					minDist = dist;
					intersectPoint = nextIntersect;
				}
				else if(minDist > dist) {
					minDist = dist;
					intersectPoint = nextIntersect;
				}
			}
		}
		return intersectPoint;
	}

	@Override
	public Vector getReflection(Vector vec, Point p) {
		// TODO Auto-generated method stub
		return null;
	}
	
}
