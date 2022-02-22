package shapes;

import java.awt.Graphics;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.StringTokenizer;

import util.MathTools;
import util.Point3D;
import shapes.Triangle3D;
import util.Vector3D;

public class Polygon3D extends Shape3D {
	
	public ArrayList<Triangle3D> triangles;
	
	public Polygon3D(String filename) {
		this.loadFromObjectFile(filename);
	}
	
	public boolean loadFromObjectFile(String filename) {
		
		BufferedReader fin = null;
		InputStream is = null;
		
		try {
			
			is = this.getClass().getResourceAsStream("/Meshes/" + filename);
			
			fin = new BufferedReader(new InputStreamReader(is));
			ArrayList<Point3D> vertices = new ArrayList<Point3D>();
			triangles = new ArrayList<Triangle3D>();
			StringTokenizer st = new StringTokenizer(fin.readLine());
			
			while(st.hasMoreTokens()) {
				
				System.out.println("LOADING");
				
				String type = st.nextToken();
				
				if(type.equals("v")) {
					double x = Double.parseDouble(st.nextToken());
					double y = Double.parseDouble(st.nextToken());
					double z = Double.parseDouble(st.nextToken());
					
					vertices.add(new Point3D(x, y, z));
				}
				else if(type.equals("f")) {
					int a = Integer.parseInt(st.nextToken()) - 1;
					int b = Integer.parseInt(st.nextToken()) - 1;
					int c = Integer.parseInt(st.nextToken()) - 1;
					
					triangles.add(new Triangle3D(new Point3D(vertices.get(a)), new Point3D(vertices.get(b)), new Point3D(vertices.get(c))));
				}
				
				String next = fin.readLine();
				if(next != null) {
					st = new StringTokenizer(next);
				}
				
			}
			fin.close();
			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return false;
		}
		
		
		
		return true;
		
	}

	@Override
	public void tick() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void draw(Graphics g) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public Point3D getIntersectPoint(Point3D p, Vector3D v) {
		double minDist = Integer.MAX_VALUE;
		Point3D minPoint = null;
		for(Triangle3D t : triangles) {
			Point3D nextPoint = t.getIntersectPoint(p, v);
			if(nextPoint != null) {
				double nextDist = MathTools.dist3D(p, nextPoint);
				if(nextDist < minDist) {
					minPoint = nextPoint;
					minDist = nextDist;
				}
			}
			
		}
		return minPoint;
	}

	public Vector3D getNormal(Point3D p, Vector3D v) {
		double minDist = Integer.MAX_VALUE;
		int minIndex = 0;
		for(int i = 0; i < triangles.size(); i++) {
			Point3D nextPoint = triangles.get(i).getIntersectPoint(p, v);
			if(nextPoint != null) {
				double nextDist = MathTools.dist3D(p, nextPoint);
				if(nextDist < minDist) {
					minIndex = i;
					minDist = nextDist;
				}
			}
		}
		return triangles.get(minIndex).normal;
	}

}
