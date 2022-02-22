package util;

public class Point implements Comparable<Point> {

	public double x, y;
	private Integer hash = null;
	
	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	public Point(Point a) {
		this.x = a.x;
		this.y = a.y;
	}
	
	public Point(Point a, Vector v) {
		this.x = a.x + v.x;
		this.y = a.y + v.y;
	}
	
	public void addVector(Vector v) {
		this.x += v.x;
		this.y += v.y;
	}
	
	public void subtractVector(Vector v) {
		this.x -= v.x;
		this.y -= v.y;
	}
	
	public void setLocation(double x, double y) {
		this.x = x;
		this.y = y;
	}
	
	@Override
	public int compareTo(Point p) {
		return this.x != p.x ? Double.compare(this.x, p.x) : Double.compare(this.y, p.y);
	}

	@Override
	public String toString() {
		return "p[" + x + ", " + y + "]";
	}

	@Override
	public int hashCode() {
		if (hash != null) {
			return hash;
		}
		return hash = hash(x, y);
	}

	public static int hash(double x, double y) {
		final int prime = 31;
		int result = 1;
		long temp;
		temp = Double.doubleToLongBits(x);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(y);
		return prime * result + (int) (temp ^ (temp >>> 32));
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (obj == null) {
			return false;
		}
		if (getClass() != obj.getClass()) {
			return false;
		}

		Point a = this;
		Point b = (Point) obj;
		if (a.x != b.x || a.y != b.y) {
			return false;
		}
		return true;
	}

	public double x() {
		return this.x;
	}
	
	public double y() {
		return this.y;
	}
	
}
