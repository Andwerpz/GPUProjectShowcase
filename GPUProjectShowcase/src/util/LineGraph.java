package util;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.util.ArrayList;

public class LineGraph {
	
	public ArrayList<Double> graph;
	
	public int x, y, width, height;
	
	public boolean fitInFrame = false;	//stretches or compresses the graph to fit it into the frame
	public boolean fitInFrameWhenFull = true;	//compresses graph to fit in frame only when graph exceeds frame limits
	
	public int xInterval = 10;
	public boolean leftToRight = true;
	
	public Color lineColor = Color.black;
	
	public boolean fixedRange = true;
	public double rangeHigh = 1;
	public double rangeLow = 0;
	
	public boolean drawFrame = true;
	public Color frameColor = Color.black;
	
	public boolean drawTicks = true;
	public Color tickColor = Color.gray;
	public int tickInterval = 10;
	public int tickLength = 10;
	
	public boolean drawValuesOnTicks = true;	//writes the number of data points for each tick. The values are drawn above the graph
	public Font font = new Font("Dialogue", 0, 12);
	
	public boolean drawYValues = true;	//draws the upper and lower bounds on the top and bottom of the left side of the graph.

	public LineGraph(int x, int y, int width, int height) {
		graph = new ArrayList<Double>();
		this.x = x; this.y = y;
		this.width = width; this.height = height;
	}
	
	public void tick() {
		
	}
	
	public void draw(Graphics g) {
		double maxY = height;
		double min = Double.MAX_VALUE;
		double max = Double.MIN_VALUE;
		for(int i = 0; i < graph.size(); i++) {
			max = Math.max(max, graph.get(i));
			min = Math.min(min, graph.get(i));
		}
		
		if(fixedRange) {
			min = rangeLow;
			max = rangeHigh;
		}
		
		if(drawYValues) {
			String minS = min + "";
			String maxS = max + "";
			int minWidth = GraphicsTools.calculateTextWidth(minS, this.font);
			int maxWidth = GraphicsTools.calculateTextWidth(maxS, this.font);
			g.drawString(minS, this.x - minWidth - 5, this.y + this.height + this.font.getSize() / 2 - 1);
			g.drawString(maxS, this.x - maxWidth - 5, this.y + this.font.getSize() / 2 - 1);
		}
		
		double xPointer = this.x;
		double curXInterval = this.xInterval;
		
		if(fitInFrame || (fitInFrameWhenFull && this.xInterval * this.graph.size() - this.xInterval > this.width)) {
			curXInterval = ((double) this.width) / ((double) graph.size() - 1);
		}
		
		g.setColor(lineColor);
		double[] yGraph = new double[graph.size()];
		for(int i = 0; i < graph.size(); i++) {
			yGraph[i] = ((graph.get(i) - min) / (max - min));
			//System.out.print(yGraph[i] + " ");
			if(i != 0) {
				g.drawLine((int) (xPointer - curXInterval), (int) (this.y + height - (yGraph[i - 1] * maxY)), (int) xPointer, (int) (this.y + height - (yGraph[i] * maxY)));
			}
			
			if((i + 1) % tickInterval == 0 && drawTicks) {
				//System.out.println("YES");
				g.drawLine((int) (xPointer), this.y, (int) (xPointer), this.y + tickLength);
			}
			
			if((i + 1) % tickInterval == 0 && drawValuesOnTicks){
				String val = (i + 1) + "";
				int width = GraphicsTools.calculateTextWidth(val, this.font);
				g.drawString(val, (int) (xPointer - width / 2), this.y - 5);
			}
			
			xPointer += curXInterval;
		}
		
		//System.out.println(max + " " + min);
		
		//System.out.println();
		//System.out.println(graph);
		
		if(drawFrame) {
			g.setColor(frameColor);
			g.drawRect(x - 1, y - 1, width + 1, height + 1);
		}
		
	}
	
}
