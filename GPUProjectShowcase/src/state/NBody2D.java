package state;

import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;

import com.aparapi.Kernel;
import com.aparapi.Range;

import main.MainPanel;
import state.NBody.Ball;
import util.MathTools;
import util.Point3D;
import util.Vector;
import util.Vector3D;

public class NBody2D extends State {

	static float xMin = -MainPanel.WIDTH / 2;
	static float xMax = MainPanel.WIDTH / 2;
	static float yMin = -MainPanel.HEIGHT / 2;
	static float yMax = MainPanel.HEIGHT / 2;

	boolean forward = false;
	boolean backward = false;
	boolean left = false;
	boolean right = false;

	float moveSpeed = 1;

	java.awt.Point prevMouse = new java.awt.Point(0, 0);
	boolean mousePressed = false;

	final NBodyKernel kernel = new NBodyKernel(Range.create(Integer.getInteger("bodies", 256), 256));

	public NBody2D(StateManager gsm) {
		super(gsm);
		// TODO Auto-generated constructor stub

		System.out.println(kernel.getTargetDevice());
	}

	public static class NBodyKernel extends Kernel {
		protected final float delT = .001f;
		protected final float espSqr = 1.0f;
		protected final float G = 1f;

		private final Range range;

		private final float[] xy; // positions xy of bodies

		private final float[] vxy; // velocity component of x,y of bodies

		private final float[] mass;
		private final float[] radius;

		@Local
		private final float[] localStuff; // local memory

		public NBodyKernel(Range _range) {
			range = _range;
			localStuff = new float[range.getLocalSize(0) * 6]; // x, y, vx, vy, mass, radius

			xy = new float[range.getGlobalSize(0) * 2];
			vxy = new float[range.getGlobalSize(0) * 2];
			mass = new float[range.getGlobalSize(0)];
			radius = new float[range.getGlobalSize(0)];
			final float maxDist = 20f;

			final float minMass = 100;
			final float maxMass = 3000;

			final float centerX = 0;
			final float centerY = 0;

			final float width = 1000;
			final float height = 1000;
			for (int body = 0; body < (range.getGlobalSize(0) * 2); body += 2) {

				// get the 3D dimensional coordinates
				xy[body + 0] = (float) (Math.random() * width - width / 2f + centerX);
				xy[body + 1] = (float) (Math.random() * height - height / 2f + centerY);

				// set mass
				mass[body / 2] = (float) (Math.random() * (maxMass - minMass) + minMass);

				// set radius
				radius[body / 2] = (float) (Math.cbrt(mass[body / 2] / Math.PI));

				// small initial velocity
//				vxy[body + 0] = (float) (Math.random() * 0.0001 * (Math.random() > 0.5 ? -1 : 1));
//				vxy[body + 1] = (float) (Math.random() * 0.0001 * (Math.random() > 0.5 ? -1 : 1));

				boolean collision = false;
				while (true) {
					for (int i = 0; i < body; i += 2) {
						float dx = xy[i + 0] - xy[body + 0];
						float dy = xy[i + 1] - xy[body + 1];
						float dist = this.sqrt(dx * dx + dy * dy);
						if (dist < radius[body / 2] + radius[i / 2]) {
							collision = true;
							System.out.println(i / 2);
							break;
						}
					}
					if (!collision) {
						break;
					}
					xy[body + 0] = (float) (Math.random() * width - width / 2f + centerX);
					xy[body + 1] = (float) (Math.random() * height - height / 2f + centerY);
					System.out.println("Collision " + body);
					collision = false;
				}

				System.out.println(body / 2);
			}
			setExplicit(true);
		}

		/**
		 * Here is the kernel entrypoint. Here is where we calculate the position of
		 * each body
		 */
		@Override
		public void run() {

			final int globalId = getGlobalId(0);

			float accx = 0.f;
			float accy = 0.f;
			final float myPosx = xy[globalId * 2 + 0];
			final float myPosy = xy[globalId * 2 + 1];

			// gravity
			for (int tile = 0; tile < (getGlobalSize(0) / getLocalSize(0)); tile++) {
				// load one tile into local memory
				final int gidx = ((tile * getLocalSize(0)) + getLocalId()) * 2;
				final int lidx = getLocalId(0) * 6;
				localStuff[lidx + 0] = xy[gidx + 0];
				localStuff[lidx + 1] = xy[gidx + 1];
				localStuff[lidx + 4] = mass[gidx / 2];
				localStuff[lidx + 5] = radius[gidx / 2];
				// Synchronize to make sure data is available for processing
				localBarrier();
				for (int i = 0; i < (getLocalSize() * 6); i += 6) {
					final float dx = localStuff[i + 0] - myPosx;
					final float dy = localStuff[i + 1] - myPosy;
					final float invDist = this.rsqrt((dx * dx) + (dy * dy) + espSqr);
					final float s = G * localStuff[i + 4] * invDist * invDist * invDist;
					accx = accx + (s * dx);
					accy = accy + (s * dy);
				}
				localBarrier();
			}
			accx = accx * delT;
			accy = accy * delT;

			// gravity accel
			vxy[globalId * 2 + 0] = vxy[globalId * 2 + 0] + accx;
			vxy[globalId * 2 + 1] = vxy[globalId * 2 + 1] + accy;

			localBarrier();

			// collision
			accx = 0f;
			accy = 0f;

			final float myVelx = vxy[globalId * 2 + 0];
			final float myVely = vxy[globalId * 2 + 1];
			final float myRadius = radius[globalId];
			final float myMass = mass[globalId];

			for (int tile = 0; tile < (getGlobalSize(0) / getLocalSize(0)); tile++) {
				// load tile into local memory
				final int gidx = ((tile * getLocalSize(0)) + getLocalId()) * 2;
				final int lidx = getLocalId(0) * 6;
				localStuff[lidx + 0] = xy[gidx + 0];
				localStuff[lidx + 1] = xy[gidx + 1];
				localStuff[lidx + 2] = vxy[gidx + 0];
				localStuff[lidx + 3] = vxy[gidx + 1];
				localStuff[lidx + 4] = mass[gidx / 2];
				localStuff[lidx + 5] = radius[gidx / 2];

				// System.out.println("Tile: " + tile + " " + globalId);

				localBarrier();

				for (int i = 0; i < (getLocalSize() * 6); i += 6) {
					// vec from my to other
					float dx = myPosx - localStuff[i + 0];
					float dy = myPosy - localStuff[i + 1];
					float dist = this.sqrt(dx * dx + dy * dy);
					int gid = (tile * getLocalSize(0)) + (i / 6);

					// check for collision, and make sure it's not self collision
					if (myRadius + localStuff[i + 5] > dist && gid != globalId) {
						// collision happened
						// get normal component of both velocities
						float normalx = dx / dist; // normalized norm vector
						float normaly = dy / dist;

						// get magnitude of velocity vectors projected along normal vector
						float aNormMag = normalx * myVelx + normaly * myVely;
						float bNormMag = normalx * localStuff[i + 2] + normaly * localStuff[i + 3];

						// 1D kinematic collision
						float aNormFinal = (aNormMag * (myMass - localStuff[i + 4]) + 2 * localStuff[i + 4] * bNormMag)
								/ (myMass + localStuff[i + 4]);

						// calculate acceleration from collision
						float dvx = normalx * (aNormFinal - aNormMag);
						float dvy = normaly * (aNormFinal - aNormMag);

						accx = accx + dvx;
						accy = accy + dvy;
					}
				}
				localBarrier();
			}

			if (accx != Float.NaN) {
				vxy[globalId * 2 + 0] = vxy[globalId * 2 + 0] + accx;
				// System.out.println(accx);
			}
			if (accy != Float.NaN) {
				vxy[globalId * 2 + 1] = vxy[globalId * 2 + 1] + accy;
			}

			// collision accel
			// System.out.println(accx + " " + accy);
			// System.out.println(vxy[globalId * 2 + 0] + " " + vxy[globalId * 2 + 1]);

			// updating pos
			xy[globalId * 2 + 0] = myPosx + (vxy[globalId * 2 + 0] * delT);
			xy[globalId * 2 + 1] = myPosy + (vxy[globalId * 2 + 1] * delT);
		}

		public void render(Graphics g) {
			// g.drawRect(100, 100, 100, 100);
//			for(float i : localStuff) {
//				System.out.println(i);
//			}
			// System.out.println(localStuff.length / 3);

			for (int i = 0; i < (range.getGlobalSize(0) * 2); i += 2) {
				float xInterval = xMax - xMin;
				float yInterval = yMax - yMin;

				int xScreen = (int) (((xy[i + 0] - xMin) / xInterval) * MainPanel.WIDTH);
				int yScreen = MainPanel.HEIGHT - (int) (((xy[i + 1] - yMin) / yInterval) * MainPanel.HEIGHT);

				g.drawOval(xScreen - (int) radius[i / 2], yScreen - (int) radius[i / 2], (int) (radius[i / 2] * 2),
						(int) (radius[i / 2] * 2));
			}
		}
	}

	@Override
	public void init() {
		// TODO Auto-generated method stub

	}

	@Override
	public void tick(Point mouse) {

		if (forward) {
			yMin += moveSpeed;
			yMax += moveSpeed;
		}
		if (backward) {
			yMin -= moveSpeed;
			yMax -= moveSpeed;
		}
		if (left) {
			xMin -= moveSpeed;
			xMax -= moveSpeed;
		}
		if (right) {
			xMin += moveSpeed;
			xMax += moveSpeed;
		}

		double xDiff = mouse.x - prevMouse.x;
		double yDiff = mouse.y - prevMouse.y;

		if (mousePressed) {
			yMin += yDiff;
			yMax += yDiff;
			xMin -= xDiff;
			xMax -= xDiff;
		}

		prevMouse.x = mouse.x;
		prevMouse.y = mouse.y;
		
		//gathering data from prev frame
		float prevEnergy = 0;
		for (int i = 0; i < kernel.range.getGlobalSize(0); i++) {
			float vx = kernel.vxy[i * 2 + 0];
			float vy = kernel.vxy[i * 2 + 1];
			float v = (float) Math.sqrt(vx * vx + vy * vy);
			prevEnergy += 0.5f * kernel.mass[i] * v * v;
			for (int j = i + 1; j < kernel.range.getGlobalSize(0); j++) {
				float dx = kernel.xy[i * 2 + 0] - kernel.xy[j * 2 + 0];
				float dy = kernel.xy[i * 2 + 1] - kernel.xy[j * 2 + 1];
				float dist = (float) Math.sqrt(dx * dx + dy * dy);	//G == 1
				prevEnergy -= kernel.mass[i] * kernel.mass[j] / dist;
			}
		}

		long time = System.currentTimeMillis();
		for (int i = 0; i < 10; i++) {
			kernel.execute(kernel.range);
		}

		if (kernel.isExplicit()) {
			kernel.get(kernel.xy);
			kernel.get(kernel.vxy);
			kernel.get(kernel.mass);
		}
		System.out.println("Time Taken: " + (System.currentTimeMillis() - time));

		// calc current frame total energy
		float px = 0;	//momentum vector
		float py = 0;
		float m = 0;	//mass sum
		float totalEnergy = 0;
		float kineticEnergy = 0;
		for (int i = 0; i < kernel.range.getGlobalSize(0); i++) {
			float vx = kernel.vxy[i * 2 + 0];
			float vy = kernel.vxy[i * 2 + 1];
			float v = (float) Math.sqrt(vx * vx + vy * vy);
			kineticEnergy += 0.5f * kernel.mass[i] * v * v;
			
			m += kernel.mass[i];
			px += kernel.mass[i] * vx;
			py += kernel.mass[i] * vy;
			
			for (int j = i + 1; j < kernel.range.getGlobalSize(0); j++) {
				float dx = kernel.xy[i * 2 + 0] - kernel.xy[j * 2 + 0];
				float dy = kernel.xy[i * 2 + 1] - kernel.xy[j * 2 + 1];
				float dist = (float) Math.sqrt(dx * dx + dy * dy);	//G == 1
				totalEnergy -= kernel.mass[i] * kernel.mass[j] / dist;
			}
		}
		totalEnergy += kineticEnergy;
		float scale = (float) Math.sqrt((kineticEnergy + prevEnergy - totalEnergy) / kineticEnergy);
		float pxScale = scale * px / m;
		float pyScale = scale * py / m;
		System.out.println(pxScale + " " + pyScale + " " + scale);
		
		for (int i = 0; i < kernel.range.getGlobalSize(0); i++) {
			kernel.vxy[i * 2 + 0] *= scale;
			kernel.vxy[i * 2 + 1] *= scale;
			//System.out.println(kernel.vxy[i * 2 + 0] + " " + kernel.vxy[i * 2 + 1]);
			
			kernel.vxy[i * 2 + 0] -= pxScale;
			kernel.vxy[i * 2 + 1] -= pyScale;
		}
		
		
		
		kernel.put(kernel.vxy);
		prevEnergy = totalEnergy;
	}

	@Override
	public void draw(Graphics g) {
		kernel.render(g);
	}

	@Override
	public void keyPressed(KeyEvent arg0) {
		int k = arg0.getKeyCode();
		if (k == KeyEvent.VK_W) {
			this.forward = true;
		} else if (k == KeyEvent.VK_S) {
			this.backward = true;
		} else if (k == KeyEvent.VK_A) {
			this.left = true;
		} else if (k == KeyEvent.VK_D) {
			this.right = true;
		}
	}

	@Override
	public void keyReleased(KeyEvent arg0) {
		int k = arg0.getKeyCode();
		if (k == KeyEvent.VK_W) {
			this.forward = false;
		} else if (k == KeyEvent.VK_S) {
			this.backward = false;
		} else if (k == KeyEvent.VK_A) {
			this.left = false;
		} else if (k == KeyEvent.VK_D) {
			this.right = false;
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

}
