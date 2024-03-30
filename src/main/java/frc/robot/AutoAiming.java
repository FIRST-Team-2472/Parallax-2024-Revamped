package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAimingConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;

public class AutoAiming extends Command {
    private double pitchAngle, yawAngle;
    private PitchMotorSubsystem pitchSubsystem;
    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private Pose2d robotPos;
    private double distanceFromSpeaker;

    

    public static double getYaw(Pose2d robotPos) {
        double yawAngle = 0;

        if (isOnBlueSide(robotPos.getX())) {
            yawAngle = flues(AutoAimingConstants.blueSpeakerPos.getX(),
                    AutoAimingConstants.blueSpeakerPos.getY(), robotPos.getX(), robotPos.getY());
        } else {
            yawAngle = flues(AutoAimingConstants.redSpeakerPos.getX(),
                    AutoAimingConstants.redSpeakerPos.getY(), robotPos.getX(), robotPos.getY());
        }

        return yawAngle;
    }

    /**
    * Returns the pitch angle for auto aiming
    * 
    * @param robotPos the robot's position
    */
    public static double getPitch(Pose2d robotPos) {
        double distanceFromSpeaker = 0;
        double pitchAngle = 0;

        if (isOnBlueSide(robotPos.getX())) {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(),
                    AutoAimingConstants.blueSpeakerPos.getX(),
                    AutoAimingConstants.blueSpeakerPos.getY());
        } else {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(),
                    AutoAimingConstants.redSpeakerPos.getX(),
                    AutoAimingConstants.redSpeakerPos.getY());
        }

        pitchAngle = distanceToAngle(distanceFromSpeaker);
        return pitchAngle;
    }

    /**
     * Returns a {@link frc.robot.AimPoint} with the pitch and yaw
     */
    public static AimPoint getAimPoint(Pose2d robotPos) {
        return new AimPoint(getYaw(robotPos), getPitch(robotPos));
    }

    /**
    * Converts the distance to the speaker into an angle
    * 
    * @param distance the distance from the robot to the target
    */ 
    public static double distanceToAngle(double distance) {
        return 102 + -31.6 * distance + 6.73 * Math.pow(distance, 2) + -0.462 * Math.pow(distance, 3);
    }

    public static double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    }

    public static boolean isOnBlueSide(double xPos) {
        return xPos < 8.25;
    }

    public static double flues(double x1, double y1, double x2, double y2) {
        return (Math.atan2((y1 - y2), (x1 - x2)) * (180 / Math.PI));
    }
}