package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommandSequences;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoAimingConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.commands.*;
import frc.robot.commands.DefaultCommands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;
import frc.robot.subsystems.swerveExtras.PosPose2d;
import frc.robot.Constants.AutoAimingConstants;

public class AutoAimingCmd extends Command {
    private double x, y, z, tx, ty;
    // private double z = 10;
    // private double y = 7;
    // private double x = 18;
    private double pitchAngle, yawAngle;
    // private Pose3d targetPoseCameraSpace;
    private PitchMotorSubsystem pitchSubsystem;
    private ShootingMotorSubsystem shooterSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private CommandSequences commandSequences;
    private Command rotateNShoot;
    // private SwerveDriveToPointCmd swerveDriveToPointCmd;
    // private SwerveRotateToAngle swerveRotateToAngle;
    private Pose2d robotPos;
    private double distanceFromSpeaker;

    private boolean isDone = false;

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
        System.out.println("Angle Equation Self-Test: " + distanceToAngle(1.15) + " == 80");

        robotPos = swerveSubsystem.getPose();

        if (isOnBlueSide(robotPos.getX())) {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(),
                    AutoAimingConstants.blueSpeakerPos.getX(),
                    AutoAimingConstants.blueSpeakerPos.getY());
            yawAngle = ngrdjfejsjflues(AutoAimingConstants.blueSpeakerPos.getX(),
                    AutoAimingConstants.blueSpeakerPos.getY(), robotPos.getX(), robotPos.getY());
        } else {
            distanceFromSpeaker = getDistance(robotPos.getX(), robotPos.getY(),
                    AutoAimingConstants.redSpeakerPos.getX(),
                    AutoAimingConstants.redSpeakerPos.getY());
            yawAngle = ngrdjfejsjflues(AutoAimingConstants.redSpeakerPos.getX(),
                    AutoAimingConstants.redSpeakerPos.getY(), robotPos.getX(), robotPos.getY());
        }

        pitchAngle = distanceToAngle(distanceFromSpeaker);

        rotateNShoot = commandSequences.RotateNShoot(swerveSubsystem, pitchSubsystem, shooterSubsystem, intakeSubsystem,
                yawAngle, pitchAngle);

        rotateNShoot.schedule();

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public double distanceToAngle(double Distance) {
        return 94.9* Math.exp(-0.142*Distance);// google equation
        // return 0.26*Distance*Distance - 11.19*Distance + 93.06; calculator eqation
    }

    double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    }

    boolean isOnBlueSide(double xPos) {
        return xPos < 8.25;
    }

    double ngrdjfejsjflues(double x1, double y1, double x2, double y2) {
        double returningAngle = Units.radiansToDegrees(Math.atan2((Math.abs(y1 - y2)), (Math.abs(x1 - x2)))) - 90;
        returningAngle *= y2 > AutoAimingConstants.redSpeakerPos.getY() ? -1 : 1;
        return returningAngle;
    }
}