package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.Constants.ArmMotorsConstants.PitchMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmSubsystems.*;
import frc.robot.commands.DefaultCommands.SwerveJoystickCmd;
import frc.robot.commands.*;
import frc.robot.commands.DefaultCommands.ArmMotorsCmd;
import frc.robot.CommandSequences;

public class RobotContainer {
  private final String placementone = "2 in amp command", placementtwo = "2 in speaker from position 2", 
  placementthree = "2 in speaker from position 1", path4 =  "2 in Speaker from Position 3", 
  testingPath =  "Drive from start", justShoot = "Just Shoot", stagePath = "Under Stage", justShootAndMove = "Shoot and Move", 
  justMovePosition2tonote2 = "Just Move from front of speaker to note 2", justRunIntake = "Run the Intake", 
  shootFromFurtherAway = "Shoot from note position", justMovePosition2tonote1 = "Move to note 1 from the front of subwoofer",
  justMovePosition2tonote3 = "Move to note 3 from front of subwoofer";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PnuematicsSubsystem pnuematicsSubsystem = new PnuematicsSubsystem();


  private final IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final PitchMotorSubsystem pitchMotorSubsystem = new PitchMotorSubsystem();
  private final ShootingMotorSubsystem shootingMotorSubsystem = new ShootingMotorSubsystem();

  private final Limelights limelights = new Limelights(swerveSubsystem, armSubsystem);
  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  
  

  public RobotContainer() {
    pitchMotorSubsystem.setDefaultCommand(new ArmMotorsCmd(pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem, () -> xbox.getLeftY(), // Pitch Motor
       () -> xbox.getRightTriggerAxis() > 0.5, () -> xbox.getLeftTriggerAxis() > 0.5, // Shooter Motors
      () -> leftJoystick.getRawButton(1),
      () -> xbox.getYButton())); // Intake Motors
  

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> -leftJoystick.getY(),
      ()-> leftJoystick.getX(),
      ()-> rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1)
    ));

    pnuematicsSubsystem.setDefaultCommand(new PnuematicsCmd(pnuematicsSubsystem));
    
    configureBindings();

    m_chooser.addOption(placementone, placementone);
    m_chooser.addOption(placementtwo, placementtwo);
    m_chooser.addOption(placementthree, placementthree);
    m_chooser.addOption(path4, path4);
    m_chooser.addOption(testingPath, testingPath);
    m_chooser.addOption(stagePath, stagePath);
    m_chooser.addOption(justShoot, justShoot);
    m_chooser.addOption(justShootAndMove, justShootAndMove);
    m_chooser.addOption(justMovePosition2tonote2, justMovePosition2tonote2);
    m_chooser.addOption(justRunIntake, justRunIntake);
    m_chooser.addOption(shootFromFurtherAway, shootFromFurtherAway);
    m_chooser.addOption(justMovePosition2tonote1, justMovePosition2tonote1);
    m_chooser.addOption(justMovePosition2tonote3, justMovePosition2tonote3);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.addCamera("Limelight Stream Intake", "limelight_intake", "mjpg:http://limelight-intake.local:5800").withSize(4,4);
    driverBoard.addCamera("Limelight Stream Shooter", "limelight_shooter", "mjpg:http://limelight-shooter.local:5800").withSize(4,4);

  }

  private void configureBindings() {
    new JoystickButton(rightJoystick, 4).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    new JoystickButton(rightJoystick, 3).onTrue(new OverrideCmd(swerveSubsystem, intakeMotorSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftBumper().onTrue(new InstantCommand(pnuematicsSubsystem :: toggleSmallpnuematics));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightBumper().onTrue(new InstantCommand(pnuematicsSubsystem :: toggleBigpnuematics));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).b().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).x().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).y().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorStandbyPresetAngle));
 
    new CommandXboxController(OperatorConstants.kXboxControllerPort).pov(0).onTrue(new InstantCommand(limelights :: scanAmpAprilTag));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).pov(180).onTrue(new InstantCommand(limelights :: scanSpeakerAprilTag));
  }

  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
       
      m_autoSelected = m_chooser.getSelected();

      if (m_autoSelected == testingPath)
        return new ParallelCommandGroup(commandSequences.driveFromZone(swerveSubsystem));

      if (m_autoSelected == placementone)
      return new ParallelCommandGroup(commandSequences.twoinampCommand(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == placementtwo)
      return new ParallelCommandGroup(commandSequences.twoinspeakerfrompositiontwoCommand(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == placementthree)
      return new ParallelCommandGroup(commandSequences.twoinspeakerfrompositiononeCommand(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == path4)
      return new ParallelCommandGroup(commandSequences.twoinspeakerfrompositionthreeCommand(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == stagePath)
      return new ParallelCommandGroup(commandSequences.underStage(swerveSubsystem));

      if (m_autoSelected == justShoot)
      return new ParallelCommandGroup(commandSequences.justShoot(pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == justShootAndMove)
      return new ParallelCommandGroup(commandSequences.justShootAndMove(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == justMovePosition2tonote2)
      return new ParallelCommandGroup(commandSequences.justMovePosition2tonote2(swerveSubsystem));

      if (m_autoSelected == justRunIntake)
      return new ParallelCommandGroup(commandSequences.justRunIntake(intakeMotorSubsystem));

      if (m_autoSelected == shootFromFurtherAway)
      return new ParallelCommandGroup(commandSequences.shootFromFurtherAway(pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == justMovePosition2tonote1)
      return new ParallelCommandGroup(commandSequences.justMovePosition2tonote1(swerveSubsystem));

      if (m_autoSelected == justMovePosition2tonote3)
      return new ParallelCommandGroup(commandSequences.justMovePosition2tonote3(swerveSubsystem));

    return null;
  }

  public void logSwerve() {

    SmartDashboard.putNumber("Heading", swerveSubsystem.getHeading());
    SmartDashboard.putString("Robot Location", swerveSubsystem.getPose().getTranslation().toString());
    SmartDashboard.putNumber("frontLeft Encoder", swerveSubsystem.getFLAbsEncoder());
    SmartDashboard.putNumber("frontRight Encoder", swerveSubsystem.getFRAbsEncoder());
    SmartDashboard.putNumber("BackLeft Encoder", swerveSubsystem.getBLAbsEncoder());
    SmartDashboard.putNumber("BackRight Encoder", swerveSubsystem.getBRAbsEncoder());
    SmartDashboard.putNumber("Shooter speed", shootingMotorSubsystem.getShooterSpeed());
    //SmartDashboard.putNumber("Arm Encoder", armSubsystem.getAbsoluteEncoder());
  }

}
