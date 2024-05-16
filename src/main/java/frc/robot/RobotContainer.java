package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmMotorsConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConstantAimToggleCmd;
import frc.robot.commands.FastAutoAimCmd;
import frc.robot.commands.OverrideCmd;
import frc.robot.commands.SetArmPitchCmd;
import frc.robot.commands.runShooter;
import frc.robot.commands.DefaultCommands.IntakeMotorCmd;
import frc.robot.commands.DefaultCommands.PitchMotorCmd;
import frc.robot.commands.DefaultCommands.PnuematicsCmd;
import frc.robot.commands.DefaultCommands.ShooterMotorsCmd;
import frc.robot.commands.DefaultCommands.SwerveJoystickCmd;
import frc.robot.subsystems.PnuematicsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.PitchMotorSubsystem;
import frc.robot.subsystems.ArmSubsystems.ShootingMotorSubsystem;

public class RobotContainer {
  private final String placementtwo = "2 in speaker from position 2", 
  placementthree = "2 in speaker from position 1", path4 =  "2 in Speaker from Position 3", 
  testingPath =  "testing path", justShoot = "Just Shoot", stagePath = "Under Stage", justShootAndMove = "Shoot and Move", 
  justMovePosition2tonote2 = "Just Move from front of speaker to note 2", justRunIntake = "Run the Intake", 
  shootFromFurtherAway = "Shoot from note position", justMovePosition2tonote1 = "Move to note 1 from the front of subwoofer",
  justMovePosition2tonote3 = "Move to note 3 from front of subwoofer", justMovePosition1tonote1 = "Move to note 1 from position 1",
  justMovePosition1tonote2 = "Move to note 2 from position 1", justMovePosition1tonote3 = "Move to note 3 from position 1",  
  fiveNoteFromPosition2 = "Five note auto collecting four notes closest to amp", threeNoteMiddle = "Three notes starting center";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //private final PnuematicsSubsystem pnuematicsSubsystem = new PnuematicsSubsystem();


  private final IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final PitchMotorSubsystem pitchMotorSubsystem = new PitchMotorSubsystem();
  private final ShootingMotorSubsystem shootingMotorSubsystem = new ShootingMotorSubsystem();

  //private final Limelights limelights = new Limelights(swerveSubsystem, armSubsystem);
  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  
  

  public RobotContainer() {
    pitchMotorSubsystem.setDefaultCommand(new PitchMotorCmd(pitchMotorSubsystem, () -> xbox.getLeftY(), () -> rightJoystick.getRawButton(1), () -> swerveSubsystem.getPose())); // Intake Motors
    intakeMotorSubsystem.setDefaultCommand(new IntakeMotorCmd(intakeMotorSubsystem, () -> rightJoystick.getRawButton(1)));
    //() -> xbox.getYButton()));
    //shootingMotorSubsystem.setDefaultCommand(new ShooterMotorsCmd(shootingMotorSubsystem, () -> xbox.getYButton(), () -> swerveSubsystem.getPose()));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> rightJoystick.getY(),
      ()-> rightJoystick.getX(),
      ()-> rightJoystick.getZ(),
      ()-> true
    ));

    //pnuematicsSubsystem.setDefaultCommand(new PnuematicsCmd(pnuematicsSubsystem));
    
    configureBindings();
    /*
    m_chooser.addOption(placementtwo, placementtwo);
    m_chooser.addOption(placementthree, placementthree);
    m_chooser.addOption(path4, path4);
    m_chooser.addOption(testingPath, testingPath);
    m_chooser.addOption(threeNoteMiddle, threeNoteMiddle);
    m_chooser.addOption(stagePath, stagePath);
    m_chooser.addOption(justShoot, justShoot);
    m_chooser.addOption(justShootAndMove, justShootAndMove);
    m_chooser.addOption(justMovePosition2tonote2, justMovePosition2tonote2);
    m_chooser.addOption(justRunIntake, justRunIntake);
    m_chooser.addOption(shootFromFurtherAway, shootFromFurtherAway);
    m_chooser.addOption(justMovePosition2tonote1, justMovePosition2tonote1);
    m_chooser.addOption(justMovePosition2tonote3, justMovePosition2tonote3);
    m_chooser.addOption(justMovePosition1tonote1, justMovePosition1tonote1);
    m_chooser.addOption(justMovePosition1tonote2, justMovePosition1tonote2);
    m_chooser.addOption(justMovePosition1tonote3, justMovePosition1tonote3);
    m_chooser.addOption(fiveNoteFromPosition2, fiveNoteFromPosition2);
    */
    //ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    //driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    //driverBoard.addCamera("Limelight Stream Intake", "limelight_intake", "mjpg:http://limelight-intake.local:5800").withSize(4,4);
    //driverBoard.addCamera("Limelight Stream Shooter", "limelight_shooter", "mjpg:http://limelight-shooter.local:5800").withSize(4,4);

  }

  private void configureBindings() {
    new JoystickButton(leftJoystick, 1).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    //new JoystickButton(rightJoystick, 3).onTrue(new OverrideCmd(swerveSubsystem, intakeMotorSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
    //new JoystickButton(rightJoystick, 13).onTrue(new InstantCommand(swerveSubsystem :: disableCams));

    //new CommandXboxController(OperatorConstants.kXboxControllerPort).leftBumper().onTrue(new InstantCommand(pnuematicsSubsystem :: toggleSmallpnuematics));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).rightBumper().onTrue(new InstantCommand(pnuematicsSubsystem :: toggleBigpnuematics));

    //new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).b().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).x().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightTrigger(0.5).onTrue(new runShooter(shootingMotorSubsystem, intakeMotorSubsystem, (leftJoystick.getRawAxis(3) + 1) * 0.2 + 0.1));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).leftTrigger(0.5).onTrue(new runShooter(shootingMotorSubsystem, intakeMotorSubsystem, 0.4, 0));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).y().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorStandbyPresetAngle));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).start().onTrue( new FastAutoAimCmd(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).back().onTrue(new ConstantAimToggleCmd(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
  }
  /*
  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
       
      m_autoSelected = m_chooser.getSelected();

      if (m_autoSelected == testingPath)
        return new ParallelCommandGroup(commandSequences.test(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if(m_autoSelected == threeNoteMiddle)
        return new ParallelCommandGroup(commandSequences.threeNoteFromPosTwo(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == placementtwo)
      return new ParallelCommandGroup(commandSequences.twoInSpeakerFromPositionTwoCommand(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == placementthree)
      return new ParallelCommandGroup(commandSequences.twoInSpeakerFromPositionOneCommand(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == path4)
      return new ParallelCommandGroup(commandSequences.twoInSpeakerFromPositionThreeCommand(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == stagePath)
      return new ParallelCommandGroup(commandSequences.underStage(swerveSubsystem));

      if (m_autoSelected == justShoot)
      return new ParallelCommandGroup(commandSequences.justShoot(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == justShootAndMove)
      return new ParallelCommandGroup(commandSequences.justShootAndMove(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == justMovePosition2tonote2)
      return new ParallelCommandGroup(commandSequences.justMovePositionTwoToNoteTwoCommand(swerveSubsystem));

      if (m_autoSelected == justRunIntake)
      return new ParallelCommandGroup(commandSequences.justRunIntake(intakeMotorSubsystem));

      if (m_autoSelected == shootFromFurtherAway)
      return new ParallelCommandGroup(commandSequences.shootFromFurtherAway(pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

      if (m_autoSelected == justMovePosition2tonote1)
      return new ParallelCommandGroup(commandSequences.justMovePositionTwoToNoteOne(swerveSubsystem));

      if (m_autoSelected == justMovePosition2tonote3)
      return new ParallelCommandGroup(commandSequences.justMovePositionTwoToNoteThreeCommand(swerveSubsystem));

      if (m_autoSelected == justMovePosition1tonote1)
      return new ParallelCommandGroup(commandSequences.justMovePositionOneToNoteOneCommand(swerveSubsystem));

      if (m_autoSelected == justMovePosition1tonote2)
      return new ParallelCommandGroup(commandSequences.justMovePositionOneToNoteTwoCommand(swerveSubsystem));

      if (m_autoSelected == justMovePosition1tonote3)
      return new ParallelCommandGroup(commandSequences.justMovePosition1tonote3(swerveSubsystem));

      if (m_autoSelected == fiveNoteFromPosition2)
      return new ParallelCommandGroup(commandSequences.fiveNoteFromPosition2(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));

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
    SmartDashboard.putNumber("Rotation", swerveSubsystem.getRotation2d().getDegrees());
    //SmartDashboard.putNumber("Arm Encoder", armSubsystem.getAbsoluteEncoder());
  }
  */
}
