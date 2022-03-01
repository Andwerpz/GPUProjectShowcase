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

	final NBodyKernel kernel = new NBodyKernel(Range.create(Integer.getInteger("bodies", 2560), 256));

	public NBody2D(StateManager gsm) {
		super(gsm);
		// TODO Auto-generated constructor stub

		System.out.println(kernel.getTargetDevice());
	}

	public static class NBodyKernel extends Kernel {
		protected final float delT = .05f;

		protected final float espSqr = 1.0f;

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

			final float width = 8000;
			final float height = 8000;
			for (int body = 0; body < (range.getGlobalSize(0) * 2); body += 2) {

				// get the 3D dimensional coordinates
				xy[body + 0] = (float) (Math.random() * width - width / 2f + centerX);
				xy[body + 1] = (float) (Math.random() * height - height / 2f + centerY);

				// set mass
				mass[body / 2] = (float) (Math.random() * (maxMass - minMass) + minMass);

				// set radius
				radius[body / 2] = (float) (Math.cbrt(mass[body / 2] / Math.PI));
				
				//small initial velocity
				vxy[body + 0] = (float) (Math.random() * 0.0001 * (Math.random() > 0.5? -1 : 1));
				vxy[body + 1] = (float) (Math.random() * 0.0001 * (Math.random() > 0.5? -1 : 1));
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
					final float s = localStuff[i + 4] * invDist * invDist * invDist;
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

						//get normal component of both velocities
						float normalx = dx / dist; // normalized norm vector
						float normaly = dy / dist;
						
						//get magnitude of velocity vectors projected along normal vector
						float aNormMag = normalx * myVelx + normaly * myVely;
						float bNormMag = normalx * localStuff[i + 2] + normaly * localStuff[i + 3];
						
						//1D kinematic collision
						float aNormFinal = (aNormMag * (myMass - localStuff[lidx + 4]) + 2 * localStuff[lidx + 4] * bNormMag) / (myMass + localStuff[lidx + 4]);
						
						//calculate acceleration from collision
						float dvx = normalx * (aNormFinal - aNormMag);
						float dvy = normaly * (aNormFinal - aNormMag);

						accx = accx + dvx;
						accy = accy + dvy;

						//System.out.println("collison");
						
//						Vector normal = new Vector(this.pos.x - c.pos.x, this.pos.y - c.pos.y);
//						normal.normalize();
//						Vector tangent = new Vector(normal);
//						tangent.rotateCounterClockwise(Math.toRadians(-90));
//						
//						System.out.println(normal.x + " " + normal.y);
//						
//						Vector aNormal = new Vector(normal);	//aNormal.setMagnitude(MathTools.dotProduct(this.vel, normal));
//						Vector bNormal = new Vector(normal);	//bNormal.setMagnitude(MathTools.dotProduct(c.vel, normal));
//						
//						double aNormTemp = MathTools.dotProduct(this.vel, normal);
//						double bNormTemp = MathTools.dotProduct(c.vel, normal);
//						
//						Vector aTangent = new Vector(tangent);	aTangent.setMagnitude(MathTools.dotProduct(this.vel, tangent));
//						Vector bTangent = new Vector(tangent);	bTangent.setMagnitude(MathTools.dotProduct(c.vel, tangent));
//						
//						System.out.println(aNormal.getMagnitude() + " " + bNormal.getMagnitude());
//						
//						//modify normal component magnitudes based on masses of objects
//						
//						double aNormalFinal = (aNormTemp * (this.mass - c.mass) + 2 * c.mass * bNormTemp) / (this.mass + c.mass);
//						double bNormalFinal = (bNormTemp * (c.mass - this.mass) + 2 * this.mass * aNormTemp) / (c.mass + this.mass);
//						
//						aNormal.setMagnitude(aNormalFinal);
//						bNormal.setMagnitude(bNormalFinal);
//						
//						//add components back together
//						Vector aFinal = new Vector(aTangent);	aFinal.addVector(aNormal);
//						Vector bFinal = new Vector(bTangent);	bFinal.addVector(bNormal);
//						
//						this.vel = new Vector(aFinal);
//						c.vel = new Vector(bFinal);

					}
				}
				localBarrier();
			}
			
			if(accx != Float.NaN) {
				vxy[globalId * 2 + 0] = vxy[globalId * 2 + 0] + accx;
				//System.out.println(accx);
			}
			if(accy != Float.NaN) {
				vxy[globalId * 2 + 1] = vxy[globalId * 2 + 1] + accy;
			}
			
			// collision accel
			//System.out.println(accx + " " + accy);
			//System.out.println(vxy[globalId * 2 + 0] + " " + vxy[globalId * 2 + 1]);

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

		long time = System.currentTimeMillis();

		kernel.execute(kernel.range);
		if (kernel.isExplicit()) {
			kernel.get(kernel.xy);
			kernel.get(kernel.vxy);
		}
		//System.out.println("Time Taken: " + (System.currentTimeMillis() - time));
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
