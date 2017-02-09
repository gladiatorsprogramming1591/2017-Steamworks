package org.usfirst.frc0.Steamworks2017.subsystems;

public class PixyPacket {
	public int X;
	public int Y;
	public int Width;
	public int Height;
	public void DumpPacket(){
		System.out.println(getPacketAsString());
	}
	public String getPacketAsString(){
		return "X = " + X + ", Y = " + Y + ", Width = " + Width + ", Height = " + Height;
	}
}