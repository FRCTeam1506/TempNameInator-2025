package frc.robot.util;

import java.awt.geom.Path2D;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldPoly {
    private FieldPoint[] points;
    private Path2D.Double polygon = new Path2D.Double();
   /*class for drawing polygons on field. ***COORDINATES MUST BE COUNTERCLOCKWISE*** */
    public FieldPoly(FieldPoint... points){
        this.points = points;
        for(int i =0;i<this.points.length;i++){
            if(i==0){
                polygon.moveTo(points[0].getX(),points[0].getY());
            } else{
                polygon.lineTo(points[i].getX(),points[i].getY());
            }
        }
        polygon.closePath();
    }
    /*start point required. ***COORDINATES MUST BE COUNTERCLOCKWISE*** */
   
    
    public boolean contains(double x,double y){
        return polygon.contains(x,y);
    }
    public boolean contains(Pose2d pos){
        return polygon.contains(pos.getX(),pos.getY());
    }
    public boolean contains(FieldPoint pos){
        return polygon.contains(pos.getX(),pos.getY());
    }
}