package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
=======
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
>>>>>>> Encoder-and-KP
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmMotorsConstants.Encoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
<<<<<<< HEAD
  private final String placementone = "Robot 1", placementtwo = "Robot 2", placementthree = "Robot 3", path4 =  "2 in Speaker from Position 2";
  
=======
  private final String placementone = "Robot 1", placementtwo = "Robot 2", placementthree = "Robot 3";

>>>>>>> Encoder-and-KP
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

<<<<<<< HEAD
=======
  Arm_Motors_Subsystem armSubsystem = new Arm_Motors_Subsystem();
>>>>>>> Encoder-and-KP

  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
<<<<<<< HEAD
  ArmMotorsSubsystem armSubsystem = new ArmMotorsSubsystem();
  private AnalogEncoder pitchMotorEncoder = new AnalogEncoder(Encoder.kEncoderPort);

  public RobotContainer() {
    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, pitchMotorEncoder, () -> xbox.getLeftY(), // Pitch Motor
      () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
=======

  public RobotContainer() {
    armSubsystem.setDefaultCommand(new ArmMotorsCmd(armSubsystem, () -> xbox.getLeftY(), // Pitch Motor
        () -> xbox.getLeftTriggerAxis() > 0.5, () -> xbox.getLeftBumper(), // Shooter Motors
>>>>>>> Encoder-and-KP
        () -> xbox.getRightTriggerAxis() > 0.5, // Push Motor
        () -> xbox.getRightBumper())); // Intake Motors

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> leftJoystick.getY(),
        () -> -leftJoystick.getX(),
        () -> -rightJoystick.getX(),
        () -> rightJoystick.getRawButton(2)));

<<<<<<< HEAD
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
    ()-> -leftJoystick.getY(),
     ()-> leftJoystick.getX(),
      ()-> rightJoystick.getX(),
       ()-> rightJoystick.getRawButton(1)
    ));
    
=======
>>>>>>> Encoder-and-KP
    configureBindings();

    m_chooser.addOption(placementone, placementone);
    m_chooser.addOption(placementtwo, placementtwo);
    m_chooser.addOption(placementthree, placementthree);
    m_chooser.addOption(path4, path4);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  private void configureBindings() {
<<<<<<< HEAD
    new JoystickButton(rightJoystick, 4).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    if (xbox.getAButton())
      resetEncoder();
  }
  public void resetEncoder(){
    pitchMotorEncoder.reset();
    pitchMotorEncoder.setDistancePerRotation(360);
=======
    new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(armSubsystem, 45));
>>>>>>> Encoder-and-KP
  }
  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
<<<<<<< HEAD
       
      m_autoSelected = m_chooser.getSelected();
    /* Replace with actual autos
      if (m_autoSelected == placementone)
=======

    m_autoSelected = m_chooser.getSelected();

    if (m_autoSelected == placementone)
>>>>>>> Encoder-and-KP
      return new ParallelCommandGroup(commandSequences.robot1Command(swerveSubsystem));

    if (m_autoSelected == placementtwo)
      return new ParallelCommandGroup(commandSequences.robot2Command(swerveSubsystem));

    if (m_autoSelected == placementthree)
      return new ParallelCommandGroup(commandSequences.robot3Command(swerveSubsystem));

      if (m_autoSelected == path4)
      return new ParallelCommandGroup(commandSequences.twoInSpeakerPosTwo(swerveSubsystem));*/

    return null;
  }

  public void logSwerve() {

    SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
    SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
    SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
    SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
    SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
    SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
  }

}
