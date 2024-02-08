package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
  private final String placementone = "Robot 1", placementtwo = "Robot 2", placementthree = "Robot 3";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static Joystick joystickL = new Joystick(OperatorConstants.kLeftJoystickControllerPort);
  public static Joystick joystickR = new Joystick(OperatorConstants.kRightJoystickControllerPort);


  Arm_Motors_Subsystem armSubsystem = new Arm_Motors_Subsystem();
  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public RobotContainer() {
    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, () -> xbox.getLeftY(), // Pitch Motor
      () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
        () -> xbox.getRightTriggerAxis() > 0.5, // Push Motor
          () -> xbox.getRightBumper())); // Intake Motors

    configureBindings();
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> joystickL.getRawAxis(OIConstants.kLeftDriverYAxis),
      () -> -joystickL.getRawAxis(OIConstants.kLeftDriverXAxis),
      () -> -joystickR.getRawAxis(OIConstants.kRightDriverRotAxis),
      () -> xbox.getRightBumper()));

    m_chooser.addOption(placementone, placementone);
    m_chooser.addOption(placementtwo, placementtwo);
    m_chooser.addOption(placementthree, placementthree);
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
       
      m_autoSelected = m_chooser.getSelected();

      if (m_autoSelected == placementone)
      return new ParallelCommandGroup(commandSequences.robot1Command(swerveSubsystem));

      if (m_autoSelected == placementtwo)
      return new ParallelCommandGroup(commandSequences.robot2Command(swerveSubsystem));

      if (m_autoSelected == placementthree)
      return new ParallelCommandGroup(commandSequences.robot3Command(swerveSubsystem));

    return null;
  }

  public void logSwerve(){
 
    SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
    SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
    SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
    SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
    SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
    SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
  }

}
