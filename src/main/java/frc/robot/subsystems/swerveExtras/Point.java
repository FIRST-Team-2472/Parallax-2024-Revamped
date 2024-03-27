package frc.robot.subsystems.swerveExtras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class Point {
    double x,y;
    Rotation2d angle;
    Point(double x, double y, double angleDegrees){
        this.x = x;
        this.y = y;
        this.angle = new Rotation2d().fromDegrees(angleDegrees);
    }
    public Pose2d getPoint(){
        return new Pose2d(x, y, angle);
    }
    public void switchToRed(){
        x = SensorConstants.sizeOfFieldMeters - x;
        angle = new Rotation2d().fromDegrees(-angle.getDegrees()+180);
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getAngle(){
        return angle.getDegrees();
    }
}