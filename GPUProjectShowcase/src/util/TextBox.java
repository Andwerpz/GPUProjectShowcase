package util;

import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.StringTokenizer;

public class TextBox {
	
	private int x, y, width, height;
	private String text;
	
	private ArrayList<String> lines;
	
	private Font font;
	
	public TextBox(int x, int y, int width, int height, String text, Font font) {
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.font = font;
		StringTokenizer st = new StringTokenizer(text);
		lines = new ArrayList<String>();
		String line = "";
		int sumSize = 0;
		while(st.hasMoreTokens()) {	//we're going to assume that every token is not wider than the text box by itself
			String next = st.nextToken();
			int curSize = GraphicsTools.calculateTextWidth(" " + next, font);
			if(sumSize + curSize > width) {
				lines.add(line);
				line = next;
				sumSize = GraphicsTools.calculateTextWidth(next, font);
			}
			else {
				line += " " + next;
				sumSize += curSize;
			}
		}
		lines.add(line);
	}
	
	public TextBox(int x, int y, int width, int height, String text) {
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.font = new Font("Dialogue", 0, 12);
		StringTokenizer st = new StringTokenizer(text);
		lines = new ArrayList<String>();
		String line = "";
		int sumSize = 0;
		while(st.hasMoreTokens()) {	//we're going to assume that every token is not wider than the text box by itself
			String next = st.nextToken();
			int curSize = GraphicsTools.calculateTextWidth(" " + next, font);
			if(sumSize + curSize > width) {
				lines.add(line);
				line = next;
				sumSize = GraphicsTools.calculateTextWidth(next, font);
			}
			else {
				line += " " + next;
				sumSize += curSize;
			}
		}
		lines.add(line);
	}
	
	public void draw(Graphics g) {
		//g.drawRect(x, y, width, height);
		int pointer = y + font.getSize();
		GraphicsTools.enableTextAntialiasing(g);
		for(int i = 0; i < lines.size(); i++) {
			g.setFont(font);
			g.drawString(lines.get(i), x, pointer);
			pointer += font.getSize() + 1;
		}
	}

}
