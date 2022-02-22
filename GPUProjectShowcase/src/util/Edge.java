package util;

public class Edge {
	
	public Point a;
	public Point b;

	public Edge(Point a, Point b) {
		boolean swap = 0 < a.compareTo(b);
		this.a = swap ? b : a;
		this.b = swap ? a : b;
	}

	public Triangle A;
	public Triangle B;

	public Triangle[] getWing() {
		if (A != null && B != null) {
			return new Triangle[] { A, B };
		}

		return new Triangle[] { A };
	}

	public void wing(Triangle t) {
		if (false) {
		} else if (this.A == null) {
			this.A = t;
		} else if (this.B == null) {
			this.B = t;
		} else {
			System.err.println("[ERR] error state in edge's wing triangle ...");
		}
	}

	@Override
	public String toString() {
		return "e[" + a + " - " + b + "]";
	}

	Integer hash = null;

	@Override
	public int hashCode() {
		if (hash != null) {
			return hash;
		}
		return hash = hash(a, b);
	}

	public static int hash(Point a, Point b) {
		int ahash = a.hashCode();
		int bhash = b.hashCode();
		return ahash * 31 + bhash;
	}

	public boolean equals(Point a, Point b) {
		if (this.a.equals(a) && this.b.equals(b)) {
			return true;
		}
		if (this.a.equals(b) && this.b.equals(a)) {
			return true;
		}
		return false;
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
		Edge A = this;
		Edge B = (Edge) obj;

		return (A.a.equals(B.a) && A.b.equals(B.b)) || (A.a.equals(B.b) && A.b.equals(B.a));
	}
}