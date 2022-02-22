package util;

import java.util.ArrayList;
import main.MainPanel;

public class MathTools {
	
	public static double dist(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
	}
	
	public static double dist3D(Point3D a, Point3D b) {
		return new Vector3D(a, b).getMagnitude();
	}
	
	public static double slope(double x1, double y1, double x2, double y2) {
		double dx = x1 - x2;
		double dy = y1 - y2;
		return dx / dy;
	}
	
	public static double radianAngleBetweenVectors(Vector a, Vector b) {
		return Math.acos(dotProduct(a, b) / (a.getMagnitude() * b.getMagnitude()));
	}
	
	// --------------- ML --------------
	
	public static double sigmoid(double x) {
		return (1d / (1d + Math.pow(Math.E,(-1d * x))));
	}
	
	//derivative of sigmoid function with center at 0
	
	public static double sigmoidDerivative(double x) {
		return sigmoid(x) * (1d - sigmoid(x));
	}
	
	public static double relu(double x) {
		return Math.max(0, x);
	}
	
	// -------------- Linear Algebra -----------
	
	public static double dotProduct(Vector a, Vector b) {
		return a.x * b.x + a.y * b.y;
	}
	
	public static double dotProduct3D(Vector3D a, Vector3D b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	
	public static double crossProduct2D(Vector a, Vector b) {
		return a.x * b.y - a.y * b.x;
	}
	
	public static Vector3D crossProduct(Vector3D a, Vector3D b) {
		
		Vector3D normal = new Vector3D(0, 0, 0);
		
		normal.x = a.y * b.z - a.z * b.y;
		normal.y = a.z * b.x - a.x * b.z;
		normal.z = a.x * b.y - a.y * b.x;
		
		return normal;
		
	}
	
	//takes in a line, lineP + lineVec, and two points, and return if the two points are on the same side of the line
	
	public static boolean pointOnSameSideOfLine(Point lineP, Vector lineVec, Point a, Point b) {
		
		//copy a and b
		Point p1 = new Point(a);
		Point p2 = new Point(b);
		
		//first offset lineP, p1, and p2 so that lineP is at the origin
		p1.subtractVector(new Vector(lineP));
		p2.subtractVector(new Vector(lineP));
		
		//calculate the line perpendicular to the input line
		//rotate the line 90 deg
		Vector perpendicular = new Vector(lineVec.y, -lineVec.x);
		
		//now take dot product of the perpendicular vector with both points
		//if both dot products are negative or positive, then the points are on the same side of the line
		Vector v1 = new Vector(p1);
		Vector v2 = new Vector(p2);
		double d1 = MathTools.dotProduct(perpendicular, v1);
		double d2 = MathTools.dotProduct(perpendicular, v2);
		
		if(d1 * d2 >= 0) {
			return true;
		}
		
		return false;
		
	}
	
	//takes in a line and a line segment, and returns where they intersect, if they intersect.
	//if they don't intersect, returns null
	
	public static Point lineLineSegmentIntersect(Point a, Point b, Point lineP, Vector lineVec) {
		
		//points: p, q
		//vectors: r, s
		//scalars: t, u 
		//p + tr = q + us
		//if p + r is the line segment and q + s is the line, only t must be bounded from 0 - 1.
		//this means we only have to solve for t, and if t is between 0 and 1, then there is an intersection at p + tr. 
		
		//solving for t
		//t = ((q - p) * s) / (r * s)
		//remember: cross product of 2d vectors gives a scalar
		Vector qp = new Vector(a, lineP);
		Vector s = new Vector(lineVec);
		Vector r = new Vector(a, b);
		
		double t = MathTools.crossProduct2D(qp, s) / MathTools.crossProduct2D(r, s);
		
		//intersection
		if(t >= 0 && t <= 1) {
			//calculate intersection point
			r.multiply(t);
			Point ans = new Point(a);
			ans.addVector(r);
			return ans;
		}
		
		//no intersection
		return null;
		
	}
	
	//assuming that the points are arranged into a convex hull.
	//get the centroid of each triangle, then take the weighted average of the centroids, using the area of each triangle as the weight.
	public static Point getCentroid(ArrayList<Point> points) {
		double accumulatedArea = 0.0f;
		double centerX = 0.0f;
		double centerY = 0.0f;

		for (int i = 0, j = points.size() - 1; i < points.size(); j = i++) {
			double temp = points.get(i).x * points.get(j).y - points.get(j).x * points.get(i).y;
			accumulatedArea += temp;
			centerX += (points.get(i).x + points.get(j).x) * temp;
			centerY += (points.get(i).y + points.get(j).y) * temp;
		}

		if (Math.abs(accumulatedArea) < 1E-7f) {
			return new Point(0, 0);
		}

		accumulatedArea *= 3f;
		return new Point(centerX / accumulatedArea, centerY / accumulatedArea);
	}
	
	// -------------- 3D graphics -------------- 
	
	//camera settings
	public static double aspectRatio = (double) MainPanel.HEIGHT / (double) MainPanel.WIDTH;
	public static double fov = Math.toRadians(90);
	public static double fovScalingFactor = 1d / Math.tan(fov * 0.5d);
	public static double zNear = 0.1;
	public static double zFar = 1000;
	public static double zNormal = zFar / (zFar - zNear);
	
	public static double[][] projectionMatrix = new double[][] {
			{aspectRatio * fovScalingFactor, 0, 0, 0},
			{0, fovScalingFactor, 0, 0},
			{0, 0, zFar / (zFar - zNear), 1},
			{0, 0, (-zNear * zNear) / (zFar - zNear), 0},
	};
	
	public static double[][] pointAtMatrix = new double[][] {};
	public static double[][] lookAtMatrix = new double[][] {};
	
	//used for scaling a point up to the window size after projection
	
	public static Point3D scalePoint(Point3D p) {
		Point3D ans = new Point3D(p);
		
		ans.x = (p.x + 1d) * (0.5 * MainPanel.WIDTH);
		ans.y = (p.y + 1d) * (0.5 * MainPanel.HEIGHT);
		
		return ans;
	}
	
	//calculates the intersection point between a line and a plane
	
	public static Point3D lineIntersectPlane(Point3D planePoint, Vector3D planeNormal, Point3D lineStart, Point3D lineEnd, double[] tRef) {
		planeNormal.normalize();
		double planeD = -MathTools.dotProduct3D(planeNormal, new Vector3D(planePoint));
		double ad = dotProduct3D(planeNormal, new Vector3D(lineStart));
		double bd = dotProduct3D(planeNormal, new Vector3D(lineEnd));
		double t = (-planeD - ad) / (bd - ad);
		Vector3D lineStartToEnd = new Vector3D(lineStart, lineEnd);
		Vector3D lineIntersect = new Vector3D(lineStartToEnd);
		lineIntersect.multiply(t);
		Point3D intersect = new Point3D(lineStart);
		intersect.addVector(lineIntersect);
		tRef[0] = t;
		return intersect;
	}
	
	//takes a triangle and clips it against the plane given. 
	//can return 0, 1, or 2 triangles.
	
	//also handles texture coordinates with depth information
	
	//as of now, doesn't really work completely. This will either completely clip a triangle, or leave it untouched. I think it's due to the bad inputs
	//yes, it was the bad inputs. When checking the normals to render, always check in real space.
	
	public static ArrayList<Point3D[]> triangleClipAgainstPlane(Point3D planePoint, Vector3D planeNormal, Point3D[] inTri, 
			Point[] inTex, ArrayList<Point[]> outTex, double[] inW, ArrayList<double[]> outW){
		
		planeNormal.normalize();
		
		ArrayList<Point3D> insidePoints = new ArrayList<Point3D>();
		ArrayList<Point3D> outsidePoints = new ArrayList<Point3D>();
		
		ArrayList<Point> insideTex = new ArrayList<Point>();
		ArrayList<Point> outsideTex = new ArrayList<Point>();
		
		ArrayList<Double> insideW = new ArrayList<Double>();
		ArrayList<Double> outsideW = new ArrayList<Double>();
		
		for(int i = 0; i < 3; i++) {
			
			Vector3D n = new Vector3D(inTri[i]);
			n.normalize();
			
			//returns signed distance from the given triangle vertice to the plane.
			//if the distance is positive, then the point lies on the "inside" of the plane
			double dist = planeNormal.x * inTri[i].x + planeNormal.y * inTri[i].y + planeNormal.z * inTri[i].z - dotProduct3D(planeNormal, new Vector3D(planePoint));
			
			if(dist >= 0) {
				insidePoints.add(inTri[i]);
				insideTex.add(inTex[i]);
				insideW.add(inW[i]);
			}
			else {
				outsidePoints.add(inTri[i]);
				outsideTex.add(inTex[i]);
				outsideW.add(inW[i]);
			}
			
		}
		
		ArrayList<Point3D[]> ans = new ArrayList<Point3D[]>();
		
		//no points are on the inside of the plane; the triangle ceases to exist.
		if(insidePoints.size() == 0) {
			return ans;
		}
		
		//the entire triangle is on the "inside" of the plane. No action needed
		if(insidePoints.size() == 3) {
			ans.add(inTri);
			outTex.add(inTex);
			outW.add(inW);
			return ans;
		}
		
		//we need to clip the triangle. As only one point lies inside the plane, this triangle can be clipped into a smaller triangle
		if(insidePoints.size() == 1) {
			
			double[] tRef1 = new double[1];
			double[] tRef2 = new double[1];
			
			//output 3d space points
			Point3D[] newTri = new Point3D[3];
			newTri[0] = insidePoints.get(0);
			newTri[1] = lineIntersectPlane(planePoint, planeNormal, insidePoints.get(0), outsidePoints.get(0), tRef1);
			newTri[2] = lineIntersectPlane(planePoint, planeNormal, insidePoints.get(0), outsidePoints.get(1), tRef2);
			
			//output texture space points
			Point[] newTex = new Point[3];
			
			Vector ab = new Vector(insideTex.get(0), outsideTex.get(0));	ab.multiply(tRef1[0]);
			Vector ac = new Vector(insideTex.get(0), outsideTex.get(1));	ac.multiply(tRef2[0]);
			
			newTex[0] = insideTex.get(0);
			newTex[1] = new Point(insideTex.get(0));	newTex[1].addVector(ab);
			newTex[2] = new Point(insideTex.get(0)); 	newTex[2].addVector(ac);
			
			//output w values
			double[] newW = new double[3];
			
			//System.out.println("TREF: " + tRef1[0]);
			
			newW[0] = insideW.get(0);
			newW[1] = insideW.get(0) + ((outsideW.get(0) - insideW.get(0)) * tRef1[0]);
			newW[2] = insideW.get(0) + ((outsideW.get(1) - insideW.get(0)) * tRef2[0]);
			
			
			outTex.add(newTex);
			outW.add(newW);
			ans.add(newTri);
			return ans;
		}
		
		//this triangle needs clipping
		//as two points lie inside the plane, we need to return 2 new triangles. 
		if(insidePoints.size() == 2) {
			
			double[] tRef1 = new double[1];
			double[] tRef2 = new double[2];
			
			
			//output new 3d space points
			Point3D[] newTri1 = new Point3D[3];
			newTri1[0] = insidePoints.get(0);
			newTri1[1] = insidePoints.get(1);
			newTri1[2] = lineIntersectPlane(planePoint, planeNormal, insidePoints.get(0), outsidePoints.get(0), tRef1);
			
			Point3D[] newTri2 = new Point3D[3];
			newTri2[0] = insidePoints.get(1);
			newTri2[1] = newTri1[2];
			newTri2[2] = lineIntersectPlane(planePoint, planeNormal, insidePoints.get(1), outsidePoints.get(0), tRef2);
			
			ans.add(newTri1);
			ans.add(newTri2);
			
			//output new texture points
			Point[] newTex1 = new Point[3];
			
			Vector ab = new Vector(insideTex.get(0), outsideTex.get(0));	ab.multiply(tRef1[0]);
			Vector cb = new Vector(insideTex.get(1), outsideTex.get(0));	cb.multiply(tRef2[0]);
			
			newTex1[0] = new Point(insideTex.get(0));
			newTex1[1] = new Point(insideTex.get(1));
			newTex1[2] = new Point(insideTex.get(0));	newTex1[2].addVector(ab);
			
			Point[] newTex2 = new Point[3];
			newTex2[0] = new Point(insideTex.get(1));
			newTex2[1] = new Point(newTex1[2]);
			newTex2[2] = new Point(insideTex.get(1));	newTex2[2].addVector(cb);
			
			outTex.add(newTex1);
			outTex.add(newTex2);
			
			//output w values
			double[] newW1 = new double[3];
			newW1[0] = insideW.get(0);
			newW1[1] = insideW.get(1);
			newW1[2] = insideW.get(0) + (outsideW.get(0) - insideW.get(0)) * tRef1[0];
			
			double[] newW2 = new double[3];
			newW2[0] = insideW.get(1);
			newW2[1] = newW1[2];
			newW2[2] = insideW.get(1) + (outsideW.get(0) - insideW.get(1)) * tRef2[0];
			
			outW.add(newW1);
			outW.add(newW2);
			
			return ans;
		}
		
		//something weird happened
		return null;
		
	}
	
	public static double[][] matrixPointAt(Point3D target, Point3D pos, Vector3D up) {
		
		
		//calculate new forward direction
		Vector3D newForward = new Vector3D(pos, target);
		newForward.normalize();
		
		//calculate new up direction
		Vector3D a = new Vector3D(newForward);
		a.multiply(dotProduct3D(up, newForward));
		Vector3D newUp = new Vector3D(a, up);
		newUp.normalize();
		
		//calculate new right direction
		Vector3D newRight = crossProduct(newUp, newForward);
		
		double[][] mat = new double[][] {
			{newRight.x, newRight.y, newRight.z, 0},
			{newUp.x, newUp.y, newUp.z, 0},
			{newForward.x, newForward.y, newForward.z, 0},
			{pos.x, pos.y, pos.z, 1}
		};
		
		return mat;
		
	}
		
	//takes in a 4x4 rotation or translation matrix and returns its inverse
		
	public static double[][] invertTransformMatrix(double[][] mat){
		
		/*	
		matrix.m[0][0] = m.m[0][0]; matrix.m[0][1] = m.m[1][0]; matrix.m[0][2] = m.m[2][0]; matrix.m[0][3] = 0.0f;
		matrix.m[1][0] = m.m[0][1]; matrix.m[1][1] = m.m[1][1]; matrix.m[1][2] = m.m[2][1]; matrix.m[1][3] = 0.0f;
		matrix.m[2][0] = m.m[0][2]; matrix.m[2][1] = m.m[1][2]; matrix.m[2][2] = m.m[2][2]; matrix.m[2][3] = 0.0f;
		matrix.m[3][0] = -(m.m[3][0] * matrix.m[0][0] + m.m[3][1] * matrix.m[1][0] + m.m[3][2] * matrix.m[2][0]);
		matrix.m[3][1] = -(m.m[3][0] * matrix.m[0][1] + m.m[3][1] * matrix.m[1][1] + m.m[3][2] * matrix.m[2][1]);
		matrix.m[3][2] = -(m.m[3][0] * matrix.m[0][2] + m.m[3][1] * matrix.m[1][2] + m.m[3][2] * matrix.m[2][2]);
		matrix.m[3][3] = 1.0f;
		*/
		
		double[][] ans = new double[][] {
			{mat[0][0], mat[1][0], mat[2][0], 0},
			{mat[0][1], mat[1][1], mat[2][1], 0},
			{mat[0][2], mat[2][1], mat[2][2], 0},
			{0, 0, 0, 0}
		};
		
		ans[3][0] = -(mat[3][0] * ans[0][0] + mat[3][1] * ans[1][0] + mat[3][2] * ans[2][0]);
		ans[3][1] = -(mat[3][0] * ans[0][1] + mat[3][1] * ans[1][1] + mat[3][2] * ans[2][1]);
		ans[3][2] = -(mat[3][0] * ans[0][2] + mat[3][1] * ans[1][2] + mat[3][2] * ans[2][2]);
		ans[3][3] = 1;
		
		return ans;
		
	}
	
	//projects point from 3d onto the 2d screen. 
	
	//stores the z buffer into the z dimension
	
	public static Point3D projectPoint(Point3D p, double[] wOut) {	
		return multiplyMatrixVector(projectionMatrix, p, wOut);
	}
	
	//Multiplies a 3D vector with a 4x4 projection matrix
	
	//it's implied that the 4th element of the vector is 1
	
	public static Point3D multiplyMatrixVector(double[][] mat, Point3D p, double[] wOut) {
		Point3D ans = new Point3D(0, 0, 0);
		
		ans.x = p.x * mat[0][0] + p.y * mat[1][0] + p.z * mat[2][0] + mat[3][0];
		ans.y = p.x * mat[0][1] + p.y * mat[1][1] + p.z * mat[2][1] + mat[3][1];
		ans.z = p.x * mat[0][2] + p.y * mat[1][2] + p.z * mat[2][2] + mat[3][2];
		double w = p.x * mat[0][3] + p.y * mat[1][3] + p.z * mat[2][3] + mat[3][3];
		
		if(w != 0) {
			ans.x /= w;
			ans.y /= w;
			ans.z /= w;
		}
		
		wOut[0] = w;
		
		return ans;
	}
	
	public static Point3D rotatePoint(Point3D p, double xRot, double yRot, double zRot) {
		Point3D p1 = new Point3D(p.x, p.y, p.z);
		
		rotateX(p1, xRot);
		rotateY(p1, yRot);
		rotateZ(p1, zRot);
		
		return p1;
	}
	
	public static void rotateX(Point3D p, double xRot) {
		double x = p.x;
		double y = p.y;
		double z = p.z;
		p.x = x;
		p.y = ((y * Math.cos(xRot)) + (z * -Math.sin(xRot)));
		p.z = ((y * Math.sin(xRot)) + (z * Math.cos(xRot)));
	}
	
	public static void rotateY(Point3D p, double yRot) {
		double x = p.x;
		double y = p.y;
		double z = p.z;
		p.x = (x * Math.cos(yRot)) + (z * Math.sin(yRot));
		p.y = y;
		p.z = (x * -Math.sin(yRot)) + (z * Math.cos(yRot));
	}
	
	public static void rotateZ(Point3D p, double zRot) {
		double x = p.x;
		double y = p.y;
		double z = p.z;
		p.x = (x * Math.cos(zRot)) + (y * -Math.sin(zRot));
		p.y = (x * Math.sin(zRot)) + (y * Math.cos(zRot));
		p.z = z;
	}

}
