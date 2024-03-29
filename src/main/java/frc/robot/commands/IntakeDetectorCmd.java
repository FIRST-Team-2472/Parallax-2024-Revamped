package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSequences;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.IntakeLimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class IntakeDetectorCmd extends Command {

    private PitchMotorSubsystem pitchSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private IntakeMotorSubsystem intakeSubsystem;
    private double tx, ty, distanceFromNote, STUPIDROBOTANGLE, otherDumbAngle, xPosDifference, yPosDifference, robotXPos, robotYPos;
    private PosPose2d notePostion;
    private Rotation2d robotRotation2d;
    private CommandSequences commandSequences;
    private Command driveNPickUpNote;

    public IntakeDetectorCmd(PitchMotorSubsystem pitchSubsystem, SwerveSubsystem swerveSubsystem, IntakeMotorSubsystem intakeSubsystem, CommandSequences commandSequences) {
        this.pitchSubsystem = pitchSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.commandSequences = commandSequences;
        addRequirements(pitchSubsystem);
    }

    @Override
    public void initialize() {
        pitchSubsystem.runPitchMotor(0,false);

        tx = LimelightHelpers.getTX("limelight-intake");
        System.out.println("tx: " + tx);
        //if (tx < 0.1 && tx > -0.1) tx = 0;
        ty = -LimelightHelpers.getTY("limelight-intake") + IntakeLimelightConstants.kIntakeLimelightTYAngleOffset;
        distanceFromNote = IntakeLimelightConstants.kIntakeLimelightHeight / Math.tan(ty);
        STUPIDROBOTANGLE = swerveSubsystem.getRotation2d().getDegrees() + tx;
        robotRotation2d = swerveSubsystem.getRotation2d().plus(Rotation2d.fromDegrees(tx));
        System.out.println("In intake detector command: " + robotRotation2d); 
        otherDumbAngle = Math.abs(STUPIDROBOTANGLE) < 90 ? 90 - Math.abs(STUPIDROBOTANGLE) : Math.abs(STUPIDROBOTANGLE) - 90;
        xPosDifference = Math.sin(otherDumbAngle) * distanceFromNote;// may need slight offset
        yPosDifference = Math.cos(otherDumbAngle) * distanceFromNote;// may need slight offset
        robotXPos = swerveSubsystem.getPose().getX();
        robotYPos = swerveSubsystem.getPose().getY();
        System.out.println("Robot's angle: " + STUPIDROBOTANGLE);
        if (STUPIDROBOTANGLE >= -1 && STUPIDROBOTANGLE <= 1){
            notePostion = new PosPose2d(robotXPos + distanceFromNote, robotYPos, robotRotation2d);

        } else if (STUPIDROBOTANGLE <= -89 && STUPIDROBOTANGLE >= -91){
            notePostion = new PosPose2d(robotXPos, robotYPos - distanceFromNote, robotRotation2d);

        } else if (STUPIDROBOTANGLE >= 179 || STUPIDROBOTANGLE <= -179){
            notePostion = new PosPose2d(robotXPos - distanceFromNote, robotYPos, robotRotation2d);

        } else if (STUPIDROBOTANGLE >= 89 && STUPIDROBOTANGLE <= 91){
            notePostion = new PosPose2d(robotXPos, robotYPos + distanceFromNote, robotRotation2d);

        } else if (STUPIDROBOTANGLE < -1 && STUPIDROBOTANGLE > -89){
            notePostion = new PosPose2d(robotXPos + xPosDifference, robotYPos - yPosDifference, robotRotation2d);

        } else if (STUPIDROBOTANGLE < -91 && STUPIDROBOTANGLE > -179){
            notePostion = new PosPose2d(robotXPos - xPosDifference, robotYPos - yPosDifference, robotRotation2d);

        } else if (STUPIDROBOTANGLE > 91 && STUPIDROBOTANGLE < 179){
            notePostion = new PosPose2d(robotXPos - xPosDifference, robotYPos + yPosDifference, robotRotation2d);

        } else if (STUPIDROBOTANGLE > 1 && STUPIDROBOTANGLE < 89){
            notePostion = new PosPose2d(robotXPos + xPosDifference, robotYPos + yPosDifference, robotRotation2d);
        } else {
            System.out.println("Bad If Stament");
        }
        System.out.println("what note postition thinks it is: " + notePostion.getRotation2d());
        System.out.println("x: " + notePostion.getX());
        System.out.println("y: " + notePostion.getY());
        driveNPickUpNote = commandSequences.driveNPickUpNote(swerveSubsystem, notePostion, intakeSubsystem);
        driveNPickUpNote.schedule(); 
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        pitchSubsystem.runPitchMotor(0);
        System.out.println("ended");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
