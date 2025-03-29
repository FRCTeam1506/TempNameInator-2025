package frc.robot.util;

public class FieldPoint {
    private double x, y;
    /*x,y point on a field for fieldPoly class */
    public FieldPoint(double x,double y){
        this.x = x;
        this.y = y;
    }
    public double getX(){
        return this.x;
    }
    public double getY(){
        return this.y;
    }
}