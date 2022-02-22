package input;

import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;

public abstract class Input {
	
	protected String name;
	
	public Input(String name) {
		this.name = name;
	}
	
	public void tick(java.awt.Point mouse) {}
	public void draw(Graphics g) {};
	
	public boolean pressed(MouseEvent arg0) {
		return false;
	}
	
	public void released(MouseEvent arg0) {}
	
	public boolean clicked(MouseEvent arg0) {
		return false;
	}
	
	public boolean hovering(java.awt.Point m) {
		return false;
	}
	
	public void keyPressed(KeyEvent arg0) {}
	
	public void keyTyped(KeyEvent arg0) {}
	
	public void keyReleased(KeyEvent arg0) {}
}
