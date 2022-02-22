package main;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.MouseInfo;
import java.awt.Point;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

import javax.swing.JPanel;
import javax.swing.SwingUtilities;


import state.StateManager;

public class MainPanel extends JPanel implements Runnable, KeyListener, MouseListener, MouseWheelListener{
	
	public final static int WIDTH = 800;
	public final static int HEIGHT = 600;
	
	private boolean isRunning = true;
	private Thread thread;
	
	private int FPS = 60;
	private long targetTime = 1000 / FPS;
	
	public Point mouse = new Point(0, 0);
	
	public static StateManager gsm;
	//private Images images;

	public MainPanel() {
		
		setPreferredSize(new Dimension(WIDTH, HEIGHT));
		
		setFocusable(true);
		setVisible(true);
		addKeyListener(this);
		addMouseListener(this);
		addMouseWheelListener(this);
		
		//this.images = new Images();
		
		this.start();
		
	}
	
	private void start() {
		thread = new Thread(this);
		thread.start();
	}
	
	@Override
	public void run() {
		
		long start, elapsed, wait;
		
		gsm = new StateManager();
		
		while(isRunning) {
			
			start = System.nanoTime();
			
			tick();
			repaint();
			
			elapsed = System.nanoTime() - start;
			wait = targetTime - elapsed / 1000000;
			
			if(wait < 0) {
				wait = 5;
			}
			
			try {
				thread.sleep(wait);
			} catch(Exception e) {
				e.printStackTrace();
			}
			
		}
		
	}
	
	public void tick() {
		
		java.awt.Point mouse2 = MouseInfo.getPointerInfo().getLocation();
		
		mouse.setLocation(mouse2.x, mouse2.y);
		SwingUtilities.convertPointToScreen(mouse2, this);
		
		mouse.setLocation(mouse.x - (mouse2.x - mouse.x), mouse.y - (mouse2.y - mouse.y));
		
		gsm.tick(mouse);
		
	}
	
	public void paintComponent(Graphics g) {
		
		super.paintComponent(g);
		
		g.clearRect(0, 0, WIDTH * 2, HEIGHT * 2);
		
		gsm.draw(g);
		
	}
	
	@Override
	public void mouseClicked(MouseEvent arg0) {
		gsm.mouseClicked(arg0);
	}

	@Override
	public void mouseEntered(MouseEvent arg0) {
		gsm.mouseEntered(arg0);
	}

	@Override
	public void mouseExited(MouseEvent arg0) {
		gsm.mouseExited(arg0);
	}

	@Override
	public void mousePressed(MouseEvent arg0) {
		gsm.mousePressed(arg0);
	}

	@Override
	public void mouseReleased(MouseEvent arg0) {
		gsm.mouseReleased(arg0);
	}

	@Override
	public void keyPressed(KeyEvent arg0) {
		gsm.keyPressed(arg0);
	}

	@Override
	public void keyReleased(KeyEvent arg0) {
		gsm.keyReleased(arg0);
	}

	@Override
	public void keyTyped(KeyEvent arg0) {
		gsm.keyTyped(arg0);
	}

	@Override
	public void mouseWheelMoved(MouseWheelEvent arg0) {
		gsm.mouseWheelMoved(arg0);
	}

	
	
}
