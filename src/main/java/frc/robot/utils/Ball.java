package frc.robot.utils;

public class Ball {
    private int id;
    private String color;
    private double centerX, centerY, radius;
    public Ball(int id, String color, double centerX, double centerY, double radius) {
        this.id = id;
        this.color = color;
        this.radius = radius;
        this.centerX = centerX;
        this.centerY = centerY;
    }
    public String toString() {
        return String.format("ID:%s, COLOR:%s, CENTERX:%s, CENTERY:%s, RADIUS:%s", this.id, this.color, this.centerX, this.centerY, this.radius);
    }
    public int getId() {return id;}
    public String getColor() {return color;}
    public double getCenterX() {return centerX;}
    public double getCenterY() {return centerY;}
    public double getRadius() {return radius;}
}
