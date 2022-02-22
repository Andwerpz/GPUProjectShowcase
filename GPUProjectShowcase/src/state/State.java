package state;

import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;

public abstract class State {

	protected StateManager gsm;
	
	public State(StateManager gsm) {
		this.gsm = gsm;
		
		init();
	}
	
	public abstract void init();
	public abstract void tick(Point mouse2);
	public abstract void draw(Graphics g);
	
	public abstract void keyPressed(KeyEvent arg0);
	public abstract void keyReleased(KeyEvent arg0);
	public abstract void keyTyped(KeyEvent arg0);
	
	public abstract void mouseClicked(MouseEvent arg0);
	public abstract void mouseEntered(MouseEvent arg0);
	public abstract void mouseExited(MouseEvent arg0);
	public abstract void mousePressed(MouseEvent arg0);
	public abstract void mouseReleased(MouseEvent arg0);
	public abstract void mouseWheelMoved(MouseWheelEvent arg0);
	
}
