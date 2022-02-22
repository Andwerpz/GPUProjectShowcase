package shapes;

import java.awt.Graphics;

import util.Point;
import util.Vector;

public abstract class Shape {

	public Shape() {};
	
	public abstract void tick();
	public abstract void draw(Graphics g);
	public abstract double getDist(util.Point p);
	public abstract Vector getReflection(Vector vec, Point p);
	
}
