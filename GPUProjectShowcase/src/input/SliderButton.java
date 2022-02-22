package input;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;

import util.GraphicsTools;

public class SliderButton extends Input{
	
	private int x, y, width, height, low, high;
	private String text;
	private Font font;
	
	private Color bodyColor;
	
	private int val;
	
	private Button head;
	
	private boolean isPressed = false;

	public SliderButton(int x, int y, int width, int height, int low, int high, String text, String name) {
		super(name);
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.low = low;
		this.high = high;
		
		this.val = low;
		
		this.head = new Button(x, y, height, height, "", name, Color.white, Color.gray);
		
		this.text = text;
		this.font = new Font("Dialogue", Font.PLAIN, 12);
		
		this.bodyColor = Color.BLACK;
		
	}
	
	public SliderButton(int x, int y, int width, int height, int low, int high, String text, String name, Color bodyColor) {
		super(name);
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.low = low;
		this.high = high;
		
		this.val = low;
		
		this.head = new Button(x, y, height, height, "", name, Color.white, bodyColor);
		
		this.text = text;
		this.font = new Font("Dialogue", Font.PLAIN, 12);
		
		this.bodyColor = bodyColor;
		
	}
	
	public boolean pressed(MouseEvent m) {
		if(head.pressed(m)) {
			isPressed = true;
			return true;
		}
		isPressed = false;
		return false;
	}
	
	public void released(MouseEvent arg0) {
		isPressed = false;
		head.released(arg0);
	}
	
	public void setText(String s) {
		this.text = s;
	}
	
	public void setVal(int n) {
		
		this.val = n;
		
		double percentFilled = ((double) width - this.height) * (val / (double) (high - low));
		
		head.setX((int) percentFilled + this.x);
		
	}
	
	public int getVal() {
		return val;
	}
	
	public Button getHead() {
		return head;
	}
	
	public void draw(Graphics g) {
		
		GraphicsTools.enableTextAntialiasing(g);
		
		g.setColor(bodyColor);
		g.fillRect(x, y, (int) head.getX() - this.x, height);
		
		g.setColor(Color.black);
		g.setFont(font);
		
		g.drawRect(x, y, width, height);
		
		g.drawString(text, x, y - (int) ((double)font.getSize() * (double)0.5));
		
		int valWidth = GraphicsTools.calculateTextWidth(val + "", this.font);
		
		g.drawString(val + "", this.x + width - valWidth, y - (int) ((double)font.getSize() * (double)0.5));
		
		//System.out.println(head.getX() - this.x);
		
		//int headOffset = width * (val / (high - low));
		
		head.draw(g);
		
	}

	public void tick(Point mouse) {
		
		if(isPressed) {
			
			int newX = (int) (mouse.x - this.height / 2);
			
			if(newX > this.x + this.width - this.height) {
				newX = this.x + this.width - this.height;
			}
			
			else if(newX < this.x) {
				newX = (this.x);
			}
			
			head.setX(newX);
			
			double percentFilled = (head.getX() - this.x) / (double)(this.width - this.height); 
			this.val = (int) (low + (high - low) * percentFilled);
			
		}
		
		
	}

}
