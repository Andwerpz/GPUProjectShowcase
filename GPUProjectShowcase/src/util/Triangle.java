package util;

import java.util.Arrays;

public class Triangle {

	public Point a;
	public Point b;
	public Point c;

	public Edge ab;
	public Edge bc;
	public Edge ca;

	private Integer hash = null;

	public Triangle(Point a, Point b, Point c) {
		Point[] tmp = { a, b, c };
		Arrays.sort(tmp);
		a = tmp[0];
		b = tmp[1];
		c = tmp[2];

		this.a = a;
		this.b = b;
		this.c = c;
	}

	public void edges(Edge ab, Edge bc, Edge ca) {
		this.ab = ab.equals(a, b) ? ab : bc.equals(a, b) ? bc : ca;
		this.bc = ab.equals(b, c) ? ab : bc.equals(b, c) ? bc : ca;
		this.ca = ab.equals(c, a) ? ab : bc.equals(c, a) ? bc : ca;
	}

	@Override
	public String toString() {
		return "t[" + a + " - " + b + " - " + c + "]";
	}

	@Override
	public int hashCode() {
		if (hash != null) {
			return hash;
		}
		return hash = hash(a, b, c);
	}

	public static int hash(Point a, Point b, Point c) {
		final int prime = 31;
		int hash = 1;
		hash = prime * hash + a.hashCode();
		hash = prime * hash + b.hashCode();
		hash = prime * hash + c.hashCode();
		return hash;
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
		Triangle A = this;
		Triangle B = (Triangle) obj;

		if (A.a.equals(B.a)) {
			return (A.b.equals(B.b) && A.c.equals(B.c)) || (A.b.equals(B.c) && A.c.equals(B.b));
		} else if (A.a.equals(B.b)) {
			return (A.b.equals(B.a) && A.c.equals(B.c)) || (A.b.equals(B.c) && A.c.equals(B.a));
		} else if (A.a.equals(B.c)) {
			return (A.b.equals(B.a) && A.c.equals(B.b)) || (A.b.equals(B.b) && A.c.equals(B.a));
		}

		return false;
	}
}