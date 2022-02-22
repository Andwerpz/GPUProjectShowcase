package input;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Toolkit;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.HashSet;

import util.GraphicsTools;

public class TextField extends Input{

	private String specialChars = "/*!@#$%^&*()\"{}_[]|\\?/<>,.";
	
	private int x, y, width, height;
	private String text;
	private String hintText;
	private boolean selected = false;
	private boolean hovered = false;
	private boolean pressed = false;
	private Font font;
	private HashSet<Integer> pressedKeys;

	private double hoveredAlpha = 0.3;
	private double selectedAlpha = 0.1;
	private double pressedAlpha = 0.5;
	private double hintTextAlpha = 0.35;

	private Color selectedColor;
	private Color textColor;

	public TextField(int x, int y, int width, String hintText, String name) {
		super(name);
		this.x = x;
		this.y = y;
		this.font = new Font("Dialogue", Font.PLAIN, 12); // default font for java swing
		this.width = width;
		this.height = 16;
		this.text = "";
		this.hintText = hintText;

		this.selectedColor = Color.gray;
		this.textColor = Color.black;

		this.pressedKeys = new HashSet<Integer>();
	}
	
	public TextField(int x, int y, int width, String hintText, String name, Font font) {
		super(name);
		this.x = x;
		this.y = y;
		this.font = font; // default font for java swing
		this.width = width;
		this.height = font.getSize() + 4;
		this.text = "";
		this.hintText = hintText;

		this.selectedColor = Color.gray;
		this.textColor = Color.black;

		this.pressedKeys = new HashSet<Integer>();
	}

	public void draw(Graphics g) {

		Graphics2D g2 = (Graphics2D) g;
		g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
		g2.setFont(font);

		if (this.pressed) {
			g2.setComposite(GraphicsTools.makeComposite(pressedAlpha));
			g2.setColor(selectedColor);
			g2.fillRect(x, y, width, height);
		}

		else if (this.selected) {
			g2.setComposite(GraphicsTools.makeComposite(selectedAlpha));
			g2.setColor(selectedColor);
			g2.fillRect(x, y, width, height);
		}

		else if (this.hovered) {
			g2.setComposite(GraphicsTools.makeComposite(hoveredAlpha));
			g2.setColor(selectedColor);
			g2.fillRect(x, y, width, height);
		}

		if (this.text.length() == 0) {
			g2.setComposite(GraphicsTools.makeComposite(hintTextAlpha));
			g2.setColor(textColor);
			g2.drawString(hintText, x + 2, y + this.font.getSize() + 2);
		}

		g2.setComposite(GraphicsTools.makeComposite(1));
		g2.setColor(textColor);
		
		g.drawRect(x, y, width, height);
		
		//drawing text into BufferedImage so that it doesn't overflow out of the text field
		BufferedImage b = new BufferedImage(this.width, this.height, BufferedImage.TYPE_INT_ARGB);
		Graphics2D gImg = (Graphics2D) b.getGraphics();
		gImg.setColor(this.textColor);
		gImg.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
		gImg.setFont(this.font);
		
		int textWidth = GraphicsTools.calculateTextWidth(text, font);
		gImg.drawString(text, 2, this.font.getSize() + 2);
		
		//adding '|' as cursor 1px after text
		if(this.selected && (System.currentTimeMillis() / 1000) % 2 == 0) {
			gImg.drawLine(2 + textWidth + 1, 2, 2 + textWidth + 1, height - 2);
		}
		
		g.drawImage(b, x, y, null);
	}

	public void setText(String text) {
		this.text = text;
	}

	public void setX(int x) {
		this.x = x;
	}

	public void setY(int y) {
		this.y = y;
	}

	public int getY() {
		return this.y;
	}

	public int getX() {
		return this.x;
	}

	public String getText() {
		return text;
	}

	public boolean getSelected() {
		return selected;
	}

	public int getWidth() {
		return width;
	}

	public int getHeight() {
		return height;
	}

	public Font getFont() {
		return font;
	}

	public Color getSelectedColor() {
		return selectedColor;
	}

	public boolean pressed(MouseEvent m) {
		Rectangle r = new Rectangle(x, y, width, height);
		if (r.contains(new Point(m.getX(), m.getY()))) {
			pressed = true;
			return true;
		}
		pressed = false;
		return false;
	}

	public boolean hovering(java.awt.Point m) {
		Rectangle r = new Rectangle(x, y, width, height);
		if (r.contains(m)) {
			hovered = true;
			return true;
		}
		this.hovered = false;
		return false;
	}

	public boolean clicked(MouseEvent arg0) {
		Rectangle r = new Rectangle(x, y, width, height);
		if (r.contains(new Point(arg0.getX(), arg0.getY()))) {
			this.selected = true;
			return true;
		}
		this.selected = false;
		pressedKeys.clear();
		return false;
	}

	public void keyPressed(KeyEvent arg0) {
		if (this.selected) {
			int k = arg0.getKeyCode();
			pressedKeys.add(k);

			// looking for ctrl + v
			if (pressedKeys.contains(KeyEvent.VK_CONTROL) && pressedKeys.contains(KeyEvent.VK_V)) {
				Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
				String result = "";
				try {
					result = (String) clipboard.getData(DataFlavor.stringFlavor);
				} catch (UnsupportedFlavorException e) {
					e.printStackTrace();
				} catch (IOException e) {
					e.printStackTrace();
				}
				this.text += result;
				return;
			}

			if (k == KeyEvent.VK_BACK_SPACE && this.text.length() != 0) {
				this.text = this.text.substring(0, this.text.length() - 1);
				return;
			}
			if (k == KeyEvent.VK_SPACE) {
				this.text += " ";
			}
			char key = arg0.getKeyChar();
			if (Character.isLetterOrDigit(key) || this.specialChars.contains((key + "").substring(0, 1))) {
				this.text += key;
			}

		}
	}

	public void keyReleased(KeyEvent arg0) {
		if (this.selected) {
			int k = arg0.getKeyCode();
			if (pressedKeys.contains(k)) {
				pressedKeys.remove(k);
			}
		}
	}

}
