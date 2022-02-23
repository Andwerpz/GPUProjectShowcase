package state;

import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.util.ArrayList;

import com.aparapi.Kernel;

import main.MainPanel;
import util.MathTools;
import util.Point3D;
import util.Vector3D;

public class NBody extends State {

	static final double g = 0.000001;

	static ArrayList<Ball> balls = new ArrayList<Ball>();

	Vector3D camera = new Vector3D(0, 0, 0);
	double xRot = 0;
	double yRot = 0;

	boolean forward = false;
	boolean backward = false;
	boolean left = false;
	boolean right = false;
	boolean up = false;
	boolean down = false;
	
	double moveSpeed = 0.1;
	
	java.awt.Point prevMouse = new java.awt.Point(0, 0);
	boolean mousePressed = false;
	
	public NBody(StateManager gsm) {
		super(gsm);

		for (int i = 0; i < 10000; i++) {
			balls.add(new Ball(Math.random() * 20, Math.random() * 20, Math.random() * 20 + 100));
		}
	}

	@Override
	public void init() {
		// TODO Auto-generated method stub

	}

	@Override
	public void tick(Point mouse) {
		
		if(forward) {
			camera.z += moveSpeed;
		}
		if(backward) {
			camera.z -= moveSpeed;
		}
		if(left) {
			camera.x -= moveSpeed;
		}
		if(right) {
			camera.x += moveSpeed;
		}
		if(up) {
			camera.y += moveSpeed;
		}
		if(down) {
			camera.y -= moveSpeed;
		}
		
		double xDiff = mouse.x - prevMouse.x;
		double yDiff = mouse.y - prevMouse.y;
		
		if(mousePressed) {
			xRot += Math.toRadians(yDiff) / 10;
			yRot += Math.toRadians(xDiff) / 10;
		}
		
		prevMouse.x = mouse.x;
		prevMouse.y = mouse.y;
		
		doGravity();
		for (Ball b : balls) {
			b.tick();
		}
	}

	public static void doGravity() {
		final double[] x = new double[balls.size()];
		final double[] y = new double[balls.size()];
		final double[] z = new double[balls.size()];

		final double[] mass = new double[balls.size()];

		final double gravConst = g;
		
		final double[] ax = new double[balls.size()];
		final double[] ay = new double[balls.size()];
		final double[] az = new double[balls.size()];

		for (int i = 0; i < balls.size(); i++) {
			x[i] = balls.get(i).pos.x;
			y[i] = balls.get(i).pos.y;
			z[i] = balls.get(i).pos.z;

			mass[i] = balls.get(i).mass;
		}
		
		Kernel kernel = new Kernel() {
			@Override
			public void run() {
				double dx,dy,dz,dist,force;
				int i = getGlobalId();
				for(int j = 0; j < x.length; j++) {
					if(j != i) {
						dx = x[j] - x[i];
						dy = y[j] - y[i];
						dz = z[j] - z[i];
		
						// calculate magnitude of acceleration
						dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
						force = gravConst * mass[j] / (dist * dist * dist);
						
						// add accel to total
						ax[i] += dx * force;
						ay[i] += dy * force;
						az[i] += dz * force;
					}
				}
			}
		};
		long time = System.currentTimeMillis();
		kernel.execute(x.length);
		System.out.println("Time Taken: " + (System.currentTimeMillis() - time));
		
		for (int i = 0; i < balls.size(); i++) {
			balls.get(i).vel.addVector(new Vector3D(ax[i], ay[i], az[i]));
		}
		
		kernel.dispose();
	}
	
	public static void doGravityCPU() {
		final double[] x = new double[balls.size()];
		final double[] y = new double[balls.size()];
		final double[] z = new double[balls.size()];

		final double[] mass = new double[balls.size()];

		final double gravConst = g;
		
		final int[] curI = new int[1];
		final double[] a = new double[3];

		for (int i = 0; i < balls.size(); i++) {
			x[i] = balls.get(i).pos.x;
			y[i] = balls.get(i).pos.y;
			z[i] = balls.get(i).pos.z;

			mass[i] = balls.get(i).mass;
		}
		
		long time = System.currentTimeMillis();
		for (int i = 0; i < balls.size(); i++) {
			curI[0] = i;
			a[0] = 0;
			a[1] = 0;
			a[2] = 0;
			double dx,dy,dz,dist,force;
			for(int j = 0; j < balls.size(); j++) {
				if(i == j) {
					continue;
				}
				
				dx = x[j] - x[i];
				dy = y[j] - y[i];
				dz = z[j] - z[i];
				
				// calculate magnitude of acceleration
				dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
				force = gravConst * mass[j] / (dist * dist * dist);
				
				// add accel to total
				a[0] += dx * force;
				a[1] += dy * force;
				a[2] += dz * force;
			}
			
			balls.get(i).vel.addVector(new Vector3D(a[0], a[1], a[2]));
		}
		System.out.println("Time Taken: " + (System.currentTimeMillis() - time));
	}

	@Override
	public void draw(Graphics g) {

		for (Ball b : balls) {

			Point3D transformed = new Point3D(b.pos);
			transformed.subtractVector(camera);
			transformed.rotateY(-yRot);
			transformed.rotateX(-xRot);

			double[] outW = new double[1];
			Point3D projected = MathTools.projectPoint(transformed, outW);

			if (outW[0] < 0) {
				continue;
			}

			Point3D scaled = MathTools.scalePoint(projected);

			g.drawRect((int) scaled.x, (int) (MainPanel.HEIGHT - scaled.y), 0, 0);
		}

	}

	@Override
	public void keyPressed(KeyEvent arg0) {
		int k = arg0.getKeyCode();
		if(k == KeyEvent.VK_W) {
			this.forward = true;
		}
		else if(k == KeyEvent.VK_S) {
			this.backward = true;
		}
		else if(k == KeyEvent.VK_A) {
			this.left = true;
		}
		else if(k == KeyEvent.VK_D) {
			this.right = true;
		}
		else if(k == KeyEvent.VK_SHIFT) {
			this.down = true;
		}
		else if(k == KeyEvent.VK_SPACE) {
			this.up = true;
		}
	}

	@Override
	public void keyReleased(KeyEvent arg0) {
		int k = arg0.getKeyCode();
		if(k == KeyEvent.VK_W) {
			this.forward = false;
		}
		else if(k == KeyEvent.VK_S) {
			this.backward = false;
		}
		else if(k == KeyEvent.VK_A) {
			this.left = false;
		}
		else if(k == KeyEvent.VK_D) {
			this.right = false;
		}
		else if(k == KeyEvent.VK_SHIFT) {
			this.down = false;
		}
		else if(k == KeyEvent.VK_SPACE) {
			this.up = false;
		}
	}

	@Override
	public void keyTyped(KeyEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseClicked(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseEntered(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mouseExited(MouseEvent arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void mousePressed(MouseEvent arg0) {
		mousePressed = true;
	}

	@Override
	public void mouseReleased(MouseEvent arg0) {
		mousePressed = false;
	}

	@Override
	public void mouseWheelMoved(MouseWheelEvent arg0) {
		// TODO Auto-generated method stub

	}

	class Ball {

		Point3D pos;
		Vector3D vel;
		double mass = 100;

		public Ball(double x, double y, double z) {
			this.pos = new Point3D(x, y, z);
			this.vel = new Vector3D(0, 0, 0);
		}

		public void tick() {
			this.pos.addVector(this.vel);
		}

	}

}
