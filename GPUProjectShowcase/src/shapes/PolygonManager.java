package shapes;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.Arrays;

import main.MainPanel;
import shapes.Line;
import util.GraphicsTools;
import util.MathTools;
import util.Point;
import util.Vector;
import shapes.Polygon;

public class PolygonManager {
	
	public ArrayList<Polygon> shapes;
	
	public double shadowAlpha = 0.5;

	public PolygonManager() {
		this.shapes = new ArrayList<Polygon>();
		
		//just as a bandaid, we need to put a big box around the play area. 
		//this is for the shadows to function correctly
		
		shapes.add(new Polygon(new ArrayList<Point>(Arrays.asList(
				new Point(-1000000, -1000000),
				new Point(1000000, -1000000),
				new Point(1000000, 1000000),
				new Point(-1000000, 1000000)))));
		
//		shapes.add(new Polygon(new ArrayList<Point>(Arrays.asList(
//				new Point(100, 400),
//				new Point(200, 400),
//				new Point(200, 500),
//				new Point(100, 500)))));
//		shapes.add(new Polygon(new ArrayList<Point>(Arrays.asList(
//				new Point(511, 111),
//				new Point(590, 109), 
//				new Point(666, 168), 
//				new Point(654, 294),
//				new Point(526, 319), 
//				new Point(561, 212)))));
		
		for(int i = -10; i <= 10; i++) {
			for(int j = -10; j <= 10; j++) {
				ArrayList<Point> nextPoints = new ArrayList<Point>();
				for(int k = 0; k < 3; k++) {
					nextPoints.add(new Point(Math.random() * 200 + i * 200, Math.random() * 200 + j * 200));
				}
				shapes.add(new Polygon(nextPoints));
			}
		}
	}
	
	public void tick() {
		
	}
	
	public void draw(Graphics g) {
		for(Polygon s : shapes) {
			s.draw(g);
		}
	}
	
	public void drawShadows(util.Point light, Graphics g, boolean debug) {
		ArrayList<Point> shadowVerticies = new ArrayList<Point>();
		
		for(int k = 0; k < this.shapes.size(); k++) {
			Polygon p = shapes.get(k);
			for(Point v : p.verticies) {
				if((v.x >= -200 && v.x <= MainPanel.WIDTH + 200 && v.y >= -200 && v.y <= MainPanel.HEIGHT + 200) || k == 0) {
					for(int i = -1; i <= 1; i ++) {
						double nextRotation = (double) i / 10;
						Vector next = new Vector(light, v);
						next.rotateCounterClockwise(Math.toRadians(nextRotation));
						
						double minDist = -1;
						Point intersectionPoint = null;
						for(Polygon ply : this.shapes) {
							Point nextIntersect = ply.getIntersectionPoint(light, next);
							if(nextIntersect != null) {
								double dist = MathTools.dist(nextIntersect.x, nextIntersect.y, light.x, light.y);
								if(minDist == -1 || minDist > dist) {
									minDist = dist;
									intersectionPoint = nextIntersect;
								}
							}
						}
						
						Point endPoint = new Point(light);
						if(intersectionPoint == null) {
							next.setMagnitude(1000);
							endPoint.addVector(next);
						}
						else {
							endPoint = intersectionPoint;
						}
						
						shadowVerticies.add(endPoint);
						
						if(debug) {
							g.drawLine((int) endPoint.x, (int) endPoint.y, (int) light.x, (int) light.y);
						}
						
					}
				}
				
				
			}
		}
		
		//sorting the points based on polar angle to mouse. you can just figure out the slope of the line from the point to the mouse, and sort them that way
		
		//or make a gimmicky comparator like i did
		shadowVerticies.sort((a, b) -> this.comparePoint(a, b, light));
		int[] xPoints = new int[shadowVerticies.size() + 7];
		int[] yPoints = new int[shadowVerticies.size() + 7];
		
		Point firstPoint = shadowVerticies.get(0);
		
		xPoints[0] = (int) firstPoint.x;
		xPoints[1] = (int) (light.x + 2000);
		xPoints[2] = (int) (light.x + 2000);
		xPoints[3] = (int) (light.x - 2000);
		xPoints[4] = (int) (light.x - 2000);
		xPoints[5] = (int) firstPoint.x;
		
		yPoints[0] = (int) (light.y - 2000);
		yPoints[1] = (int) (light.y - 2000);
		yPoints[2] = (int) (light.y + 2000);
		yPoints[3] = (int) (light.y + 2000);
		yPoints[4] = (int) (light.y - 2000);
		yPoints[5] = (int) (light.y - 2000);
		
		int pointer = 6;
		for(int i = 0; i < shadowVerticies.size(); i++) {
			xPoints[pointer] = (int) shadowVerticies.get(i).x;
			yPoints[pointer] = (int) shadowVerticies.get(i).y;
			pointer ++;
			//g.drawString(i + "", xPoints[i], yPoints[i]);
		}
		
		xPoints[xPoints.length - 1] = (int) firstPoint.x;
		yPoints[yPoints.length - 1] = (int) firstPoint.y;
		
		java.awt.Polygon shadow = new java.awt.Polygon(xPoints, yPoints, xPoints.length);
		
		Graphics2D g2 = (Graphics2D) g;
		g2.setColor(Color.black);
		g2.setComposite(GraphicsTools.makeComposite(this.shadowAlpha));
		g2.fillPolygon(shadow);
		g2.setComposite(GraphicsTools.makeComposite(1));
	}
	
	public int comparePoint(Point a, Point b, Point light) {
		if(a.x > light.x && b.x < light.x) {
			return 1;
		}
		else if(a.x < light.x && b.x > light.x) {
			return -1;
		}
		else if(a.x > light.x && b.x > light.x) {
			double da = (a.y - light.y) / (a.x - light.x);
			double db = (b.y - light.y) / (b.x - light.x); 
			if(da < db) {
				return 1;
			}
			else {
				return -1;
			}
		}
		else if(a.x < light.x && b.x < light.x) {
			double da = (a.y - light.y) / (a.x - light.x);
			double db = (b.y - light.y) / (b.x - light.x); 
			
			if(da < db) {
				return 1;
			}
			else {
				return -1;
			}
		}
		else if(a.y > b.y) {
			return 1;
		}
		return 1;
	}
	
	public void move(double xDiff, double yDiff) {
		for(Polygon s : shapes) {
			for(Line l : s.lines) {
				l.x1 += xDiff;
				l.x2 += xDiff;
				l.y1 += yDiff;
				l.y2 += yDiff;
			}
			for(Point p : s.verticies) {
				p.x += xDiff;
				p.y += yDiff;
			}
		}
	}
	
}
