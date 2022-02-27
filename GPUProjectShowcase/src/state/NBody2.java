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
import util.Vector3D;

public class NBody2 extends State{
	
	static Vector3D camera = new Vector3D(0, 0, -20);
	static double xRot = 0;
	static double yRot = 0;

	boolean forward = false;
	boolean backward = false;
	boolean left = false;
	boolean right = false;
	boolean up = false;
	boolean down = false;

	double moveSpeed = 1;

	java.awt.Point prevMouse = new java.awt.Point(0, 0);
	boolean mousePressed = false;

	final NBodyKernel kernel = new NBodyKernel(Range.create(Integer.getInteger("bodies", 25600), 256));
	
	public NBody2(StateManager gsm) {
		super(gsm);
		// TODO Auto-generated constructor stub
		
		
	}

	public static class NBodyKernel extends Kernel {   
		protected final float delT = .005f;

		protected final float espSqr = 1.0f;

		protected final float mass = .1f;

		private final Range range;

		private final float[] xyz; // positions xy and z of bodies

		private final float[] vxyz; // velocity component of x,y and z of bodies

		@Local
		private final float[] localStuff; // local memory

		public NBodyKernel(Range _range) {
			range = _range;
			localStuff = new float[range.getLocalSize(0) * 3];

			xyz = new float[range.getGlobalSize(0) * 3];
			vxyz = new float[range.getGlobalSize(0) * 3];
			final float maxDist = 20f;
			
			final float centerX = 0;
			final float centerY = 0;
			final float centerZ = 0;
			
			final float width = 100;
			final float height = 100;
			final float depth = 100;
			for (int body = 0; body < (range.getGlobalSize(0) * 3); body += 3) {

				// get the 3D dimensional coordinates
				xyz[body + 0] = (float) (Math.random() * width - width / 2f + centerX);
				xyz[body + 1] = (float) (Math.random() * height - height / 2f + centerY);
				xyz[body + 2] = (float) (Math.random() * depth - depth / 2f + centerZ);
			}
			setExplicit(true);
		}

		/**
		 * Here is the kernel entrypoint. Here is where we calculate the position of
		 * each body
		 */
		@Override
		public void run() {

			final int globalId = getGlobalId(0) * 3;

			float accx = 0.f;
			float accy = 0.f;
			float accz = 0.f;
			final float myPosx = xyz[globalId + 0];
			final float myPosy = xyz[globalId + 1];
			final float myPosz = xyz[globalId + 2];

			for (int tile = 0; tile < (getGlobalSize(0) / getLocalSize(0)); tile++) {
				// load one tile into local memory
				final int gidx = ((tile * getLocalSize(0)) + getLocalId()) * 3;
				final int lidx = getLocalId(0) * 3;
				localStuff[lidx + 0] = xyz[gidx + 0];
				localStuff[lidx + 1] = xyz[gidx + 1];
				localStuff[lidx + 2] = xyz[gidx + 2];
				// Synchronize to make sure data is available for processing
				localBarrier();

				for (int i = 0; i < (getLocalSize() * 3); i += 3) {
					final float dx = localStuff[i + 0] - myPosx;
					final float dy = localStuff[i + 1] - myPosy;
					final float dz = localStuff[i + 2] - myPosz;
					final float invDist = rsqrt((dx * dx) + (dy * dy) + (dz * dz) + espSqr);
					final float s = mass * invDist * invDist * invDist;
					accx = accx + (s * dx);
					accy = accy + (s * dy);
					accz = accz + (s * dz);
				}
				localBarrier();
			}
			accx = accx * delT;
			accy = accy * delT;
			accz = accz * delT;
			xyz[globalId + 0] = myPosx + (vxyz[globalId + 0] * delT) + (accx * .5f * delT);
			xyz[globalId + 1] = myPosy + (vxyz[globalId + 1] * delT) + (accy * .5f * delT);
			xyz[globalId + 2] = myPosz + (vxyz[globalId + 2] * delT) + (accz * .5f * delT);

			vxyz[globalId + 0] = vxyz[globalId + 0] + accx;
			vxyz[globalId + 1] = vxyz[globalId + 1] + accy;
			vxyz[globalId + 2] = vxyz[globalId + 2] + accz;

		}
		
		public void render(Graphics g) {
			//g.drawRect(100, 100, 100, 100);
//			for(float i : localStuff) {
//				System.out.println(i);
//			}
			System.out.println(localStuff.length / 3);
			
			for (int i = 0; i < (range.getGlobalSize(0) * 3); i += 3) {

				Point3D transformed = new Point3D(xyz[i + 0], xyz[i + 1], xyz[i + 2]);
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
	}

	@Override
	public void init() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void tick(Point mouse) {
		Vector3D forwardVec = new Vector3D(0, 0, 1);
		forwardVec.rotateY(yRot);

		if (forward) {
			camera.addVector(forwardVec);
		}
		if (backward) {
			camera.subtractVector(forwardVec);
		}
		forwardVec.rotateY(Math.toRadians(90));
		if (left) {
			camera.subtractVector(forwardVec);
		}
		if (right) {
			camera.addVector(forwardVec);
		}
		if (up) {
			camera.y += moveSpeed;
		}
		if (down) {
			camera.y -= moveSpeed;
		}

		double xDiff = mouse.x - prevMouse.x;
		double yDiff = mouse.y - prevMouse.y;

		if (mousePressed) {
			xRot += Math.toRadians(yDiff) / 10;
			yRot += Math.toRadians(xDiff) / 10;
		}

		prevMouse.x = mouse.x;
		prevMouse.y = mouse.y;
		
		kernel.execute(kernel.range);
        if (kernel.isExplicit()) {
           kernel.get(kernel.xyz);
        }
		
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
		} else if (k == KeyEvent.VK_SHIFT) {
			this.down = true;
		} else if (k == KeyEvent.VK_SPACE) {
			this.up = true;
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
		} else if (k == KeyEvent.VK_SHIFT) {
			this.down = false;
		} else if (k == KeyEvent.VK_SPACE) {
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
	
}
