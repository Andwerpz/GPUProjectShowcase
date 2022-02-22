package util;

import java.awt.Color;
import java.awt.Component;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.image.BufferedImage;

import input.InputManager;

public abstract class ScrollWindow {

	protected InputManager im;

	protected int x, y, width, height;

	protected int offset = 0; // offset for which portion of the window to display
	protected int realHeight; // total height of the window

	protected java.awt.Point mouse = new java.awt.Point(0, 0);

	public ScrollWindow(int x, int y, int width, int height, int realHeight) {
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.realHeight = realHeight;
		this.im = new InputManager();
	}

	// initializing buffered image for repaint to draw on
	public void draw(Graphics g) {
		BufferedImage b = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
		Graphics gImg = b.getGraphics();
		gImg.translate(0, -offset);
		this.repaint(gImg, b);

		// drawing outline
		g.setColor(Color.BLACK);
		g.drawRect(x - 1, y - 1, width + 1, height + 1);

		// drawing img
		g.drawImage(b, x, y, null);

		// draw scroll bar
		int scrollBarHeight = (int) (Math.min(1, (double) this.height / (double) this.realHeight) * this.height);
		if (scrollBarHeight == 1) {
			return;
		}
		int scrollBarWidth = 10;
		int scrollBarY = (int) ((this.height - scrollBarHeight)
				* ((double) this.offset / (double) (this.realHeight - this.height)));
		Graphics2D g2 = (Graphics2D) g;
		g2.setComposite(GraphicsTools.makeComposite(0.25));
		g2.fillRect(x + width - scrollBarWidth, y + scrollBarY, scrollBarWidth, scrollBarHeight);
		g2.setComposite(GraphicsTools.makeComposite(1));
	}

	// doing the actual drawing
	// the graphics object has already been offset, draw relative to 0, 0
	public abstract void repaint(Graphics g, BufferedImage b);

	public void tick(java.awt.Point mouse) {
		this.mouse = mouse;
		im.tick(convertPoint(mouse));
	}

	public void setRealHeight(int height) {
		this.realHeight = height;
	}

	public int getRealHeight() {
		return this.realHeight;
	}

	// returns true if the point is contained within the frame of the window
	public boolean containsPoint(java.awt.Point p) {
		Rectangle rect = new Rectangle(x, y, width, height);
		return rect.contains(mouse);
	}

	// converts the point so that it is relative to the top left corner of this
	// window
	public java.awt.Point convertPoint(java.awt.Point p) {
		return new java.awt.Point(p.x - this.x, p.y - this.y + this.offset);
	}

	// converts mouse event so that it is relative to top left corner of window
	public MouseEvent convertMouseEvent(MouseEvent arg0) {
		return new MouseEvent((Component) arg0.getSource(), arg0.getID(), arg0.getWhen(), arg0.getModifiers(),
				arg0.getX() - this.x, arg0.getY() - this.y + this.offset, arg0.getClickCount(), false);
	}

	public void keyPressed(KeyEvent arg0) {
		im.keyPressed(arg0);
	}

	public void keyReleased(KeyEvent arg0) {
		im.keyReleased(arg0);
	}

	public void keyTyped(KeyEvent arg0) {
		im.keyTyped(arg0);
	}

	public void mouseClicked(MouseEvent arg0) {

	}

	public void mouseEntered(MouseEvent arg0) {

	}

	public void mouseExited(MouseEvent arg0) {

	}

	public void mousePressed(MouseEvent arg0) {
		if (this.containsPoint(mouse)) {
			im.mousePressed(convertMouseEvent(arg0));
		}

	}

	public void mouseReleased(MouseEvent arg0) {
		im.mouseReleased(convertMouseEvent(arg0));
	}

	public void mouseWheelMoved(MouseWheelEvent arg0) {
		if (this.containsPoint(mouse)) {
			offset += arg0.getUnitsToScroll();
			offset = Math.min(realHeight - height, offset);
			offset = Math.max(0, offset);
			// System.out.println(arg0.getUnitsToScroll() + " " + offset);
		}
	}

}
