package main;

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;

import javax.swing.JFrame;

public class Main {

	public static void main(String[] args) {
		
		GraphicsEnvironment graphics =
		GraphicsEnvironment.getLocalGraphicsEnvironment();
		GraphicsDevice device = graphics.getDefaultScreenDevice();
		
		JFrame frame = new JFrame();
		
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setResizable(false);
		frame.setLayout(new BorderLayout());
		frame.add(new MainPanel(), BorderLayout.CENTER);
		frame.pack();
		frame.setLocationRelativeTo(null);
		
		frame.setVisible(true);
		
	}
	
}
