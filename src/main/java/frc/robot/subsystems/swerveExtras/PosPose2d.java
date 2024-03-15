package frc.robot.subsystems.swerveExtras;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class PosPose2d{
    // positive/blue position -> simplfyed position for comparisions to auto points
    
    double x,y;
    Rotation2d angle;
    
    public PosPose2d(double x, double y, double angle){ // red is true
        this.y = y;
        this.x = x;
        this.angle = new Rotation2d(Math.toRadians(angle));
    }
    
    public double getX(){
        return x;
    }

    public void changeToRed(){
        x = Constants.SensorConstants.sizeOfFieldMeters - x;
        angle = new Rotation2d(180 - angle.getDegrees());
    }
    
    public double getY(){
        return y;
    }
    
    public Rotation2d getAngle(){
        return angle;
    }
}
