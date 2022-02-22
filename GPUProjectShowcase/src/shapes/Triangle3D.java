package shapes;

import java.awt.Graphics;

import util.MathTools;
import util.Point3D;
import util.Vector3D;

public class Triangle3D extends Shape3D {
	
	public Point3D a, b, c;	//try to instantiate in clockwise order, so calculating normals are easy
	
	public Vector3D normal;
	
	public Triangle3D(Point3D a, Point3D b, Point3D c) {
		this.a = new Point3D(a);
		this.b = new Point3D(b);
		this.c = new Point3D(c);
		this.normal = this.calculateNormal();
	}

	public void tick() {
		
	}

	public void draw(Graphics g) {
		
	}
	
	public Vector3D calculateNormal() {
		Point3D vertex0 = new Point3D(a);
        Point3D vertex1 = new Point3D(b);
        Point3D vertex2 = new Point3D(c);
        Vector3D edge1 = new Vector3D(vertex1, vertex0);
        Vector3D edge2 = new Vector3D(vertex2, vertex0);
        return MathTools.crossProduct(edge1, edge2);
	}
	
	private static final double EPSILON = 0.0001;
	
	//taken from wikipedia. 
	//moller trumbore intersection algorithm
	
	//returns the intersection point between this triangle and a 3dvector if they intersect, null if they dont

    public Point3D getIntersectPoint(Point3D rayOrigin, Vector3D rayVector) {
    	rayVector.normalize();
    	//System.out.println("YES");
        Point3D vertex0 = new Point3D(a);
        Point3D vertex1 = new Point3D(b);
        Point3D vertex2 = new Point3D(c);
        Vector3D edge1 = new Vector3D(vertex1.x, vertex1.y, vertex1.z);
        edge1.addVector(new Vector3D(-vertex0.x, -vertex0.y, -vertex0.z));
        Vector3D edge2 = new Vector3D(vertex2.x, vertex2.y, vertex2.z);
        edge2.addVector(new Vector3D(-vertex0.x, -vertex0.y, -vertex0.z));
        Vector3D h = MathTools.crossProduct(rayVector, edge2);
        Vector3D s = new Vector3D(0, 0, 0);
        Vector3D q = new Vector3D(0, 0, 0);
        double a, f, u, v;
        //h.cross(rayVector, edge2);
        a = MathTools.dotProduct3D(h, edge1);
        if (a > -EPSILON && a < EPSILON) {
        	//System.out.println('1');
            return null;    // This ray is parallel to this triangle.
        }
        f = 1.0 / a;
        s = new Vector3D(rayOrigin.x, rayOrigin.y, rayOrigin.z);
        s.addVector(new Vector3D(-vertex0.x, -vertex0.y, -vertex0.z));
        //s.sub(rayOrigin, vertex0);
        u = f * (MathTools.dotProduct3D(s, h));
        //u = f * (s.dot(h));
        if (u < 0.0 || u > 1.0) {
        	//System.out.println('2');
            return null;
        }
        //q.cross(s, edge1);
        q = MathTools.crossProduct(s, edge1);
        v = f * MathTools.dotProduct3D(rayVector, q);
        if (v < 0.0 || u + v > 1.0) {
        	//System.out.println('3');
            return null;
        }
        // At this stage we can compute t to find out where the intersection point is on the line.
        //double t = f * edge2.dot(q);
        double t = f * MathTools.dotProduct3D(edge2, q);
        if (t > EPSILON) // ray intersection
        {
            Point3D outIntersectionPoint = new Point3D(rayOrigin);
            //outIntersectionPoint.scaleAdd(t, rayVector, rayOrigin);
            rayVector.multiply(t);
            outIntersectionPoint.addVector(rayVector);
            
            //System.out.println("YES");
            
            return outIntersectionPoint;
        } else // This means that there is a line intersection but not a ray intersection.
        {
            return null;
        }
    }

}
