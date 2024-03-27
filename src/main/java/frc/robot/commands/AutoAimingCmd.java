package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSequences;
import frc.robot.Constants.AutoAimingConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;


public class AutoAimingCmd extends Command {
    private double pitchAngle, yawAngle;
    private PitchMotorSubsystem pitchSubsystem;
    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private CommandSequences commandSequences;
    private Command rotateNShoot;
    private Pose2d robotPos;
    private double distanceFromSpeaker;

    public AutoAimingCmd(SwerveSubsystem swerveSubsystem, PitchMotorSubsystem pitchSubsystem,
            ShootingMotorSubsystem shooterSubsystem,
            IntakeMotorSubsystem intakeSubsystem, CommandSequences commandSequences) {

        addRequirements(swerveSubsystem);
        addRequirements(pitchSubsystem);
        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);

        this.pitchSubsystem = pitchSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.commandSequences = commandSequences;

    }

    @Override
    public void initialize() {
        System.out.println("Distance Self-Test: " + getDistance(2.2, 5.6, 0.25, 5.6) + " == 1.95");
        System.out.println("Angle Equation Self-Test: " + distanceToAngle(1.15) + " == 80.858");

        robotPos = swerveSubsystem.getPose();

        if (isOnBlueSide(robotPos.getX())) {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(),
                    AutoAimingConstants.blueSpeakerPos.getX(),
                    AutoAimingConstants.blueSpeakerPos.getY());
            yawAngle = flues(AutoAimingConstants.blueSpeakerPos.getX(),
                    AutoAimingConstants.blueSpeakerPos.getY(), robotPos.getX(), robotPos.getY());
        } else {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(),
                    AutoAimingConstants.redSpeakerPos.getX(),
                    AutoAimingConstants.redSpeakerPos.getY());
            yawAngle = flues(AutoAimingConstants.redSpeakerPos.getX(),
                    AutoAimingConstants.redSpeakerPos.getY(), robotPos.getX(), robotPos.getY());
        }

        pitchAngle = distanceToAngle(distanceFromSpeaker);

        rotateNShoot = commandSequences.RotateNShoot(swerveSubsystem, pitchSubsystem, shooterSubsystem, intakeSubsystem,
                yawAngle, pitchAngle);

        rotateNShoot.schedule();

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public double distanceToAngle(double distance) {
        return 109 + -31.6 * distance + 6.73 * Math.pow(distance, 2) + -0.462 * Math.pow(distance, 3);
    }

    double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    }

    boolean isOnBlueSide(double xPos) {
        return xPos < 8.25;
    }

    double flues(double x1, double y1, double x2, double y2) {
        return -(Math.atan2((y1 - y2), (x1 - x2)) * (180 / Math.PI));
    }

    /* public static double ngrdjfejsjflues(double x1, double y1, double x2, double y2) {
        // Calculate the difference in x and y coordinates
        double dx = x2 - x1;
        double dy = y2 - y1;

        // Use atan2 to calculate the angle in radians (more accurate than atan)
        double angleInRadians = Math.atan2(dy, dx);

        // Convert from radians to degrees
        double angleInDegrees = Math.toDegrees(angleInRadians);

        return angleInDegrees;
    } */
}