package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConstantAimToggleCmd;
import frc.robot.commands.FastAutoAimCmd;
import frc.robot.commands.OverrideCmd;
import frc.robot.commands.SetArmPitchCmd;
import frc.robot.commands.SwerveRotateToAngle;
import frc.robot.commands.runIntake;
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
  private final String SPtwoNtwoNone = "PP: Three Note Auto from position 2 to note 2 to note 1", SPtwoNtwo = "PP: two note in speaker from position 2 to note 2",
  SPtwoNoneNtwoNthree = "PP: 4 in speaker from position 2", SPtwoNthreeNtwoNoneNfour = "PP: 4 in speaker from position 2 + collect one more",
  SPtwoNoneNfour = "PP: 3 in speaker from position 2 Note 1 - 4", SPtwoNtwoNfour = "PP: 3 in speaker from position 2 - Note 2 - 4";
  
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PnuematicsSubsystem pnuematicsSubsystem = new PnuematicsSubsystem();


  private final IntakeMotorSubsystem intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final PitchMotorSubsystem pitchMotorSubsystem = new PitchMotorSubsystem();
  private final ShootingMotorSubsystem shootingMotorSubsystem = new ShootingMotorSubsystem();

  //private final Limelights limelights = new Limelights(swerveSubsystem, armSubsystem);
  XboxController xbox = new XboxController(OperatorConstants.kXboxControllerPort);
  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoyPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoyPort);
  
  

  public RobotContainer() {
    pitchMotorSubsystem.setDefaultCommand(new PitchMotorCmd(pitchMotorSubsystem, () -> xbox.getLeftY(), () -> leftJoystick.getRawButton(1), () -> swerveSubsystem.getPose())); // Intake Motors
    intakeMotorSubsystem.setDefaultCommand(new IntakeMotorCmd(intakeMotorSubsystem, () -> leftJoystick.getRawButton(1),
    () -> xbox.getYButton()));
    shootingMotorSubsystem.setDefaultCommand(new ShooterMotorsCmd(shootingMotorSubsystem, () -> xbox.getYButton(), () -> swerveSubsystem.getPose()));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, 
      ()-> leftJoystick.getY(),
      ()-> leftJoystick.getX(),
      ()-> -rightJoystick.getX(),
      ()-> rightJoystick.getRawButton(1)
    ));

    pnuematicsSubsystem.setDefaultCommand(new PnuematicsCmd(pnuematicsSubsystem));
    
    configureBindings();


    m_chooser.addOption(SPtwoNtwoNone, SPtwoNtwoNone);
    m_chooser.addOption(SPtwoNtwo, SPtwoNtwo);
    m_chooser.addOption(SPtwoNoneNtwoNthree, SPtwoNoneNtwoNthree);
    m_chooser.addOption(SPtwoNthreeNtwoNoneNfour, SPtwoNthreeNtwoNoneNfour);
    m_chooser.addOption(SPtwoNoneNfour, SPtwoNoneNfour);
    m_chooser.addOption(SPtwoNtwoNfour, SPtwoNtwoNfour);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    driverBoard.addCamera("Limelight Stream Intake", "limelight_intake", "mjpg:http://limelight-intake.local:5800").withSize(4,4);
    driverBoard.addCamera("Limelight Stream Shooter", "limelight_shooter", "mjpg:http://limelight-shooter.local:5800").withSize(4,4);

    //warning a name change will break auto paths because pathplanner will not update it
    NamedCommands.registerCommand("runIntake", new runIntake(intakeMotorSubsystem, 0, 5, pitchMotorSubsystem));
    NamedCommands.registerCommand("Shoot", new runShooter(shootingMotorSubsystem, intakeMotorSubsystem, .9 ));
    NamedCommands.registerCommand("autoShoot", new FastAutoAimCmd(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));
    NamedCommands.registerCommand("angle to speaker", new SetArmPitchCmd(pitchMotorSubsystem, Constants.ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    NamedCommands.registerCommand("rotate to -90", new SwerveRotateToAngle(swerveSubsystem, CommandSequences.teamChangeAngle(-90)));

    AutoBuilder.configureHolonomic(
            () -> swerveSubsystem.getPose(), // Robot pose supplier for auot (correct range -180-180)
            swerveSubsystem ::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            () -> swerveSubsystem.getChassisSpeedsRobotRelative(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveSubsystem :: runModulesRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
            () -> SwerveSubsystem.isOnRed(),
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            swerveSubsystem // Reference to this subsystem to set requirements
        );
  }

  private void configureBindings() {
    new JoystickButton(rightJoystick, 4).onTrue(new InstantCommand(swerveSubsystem :: zeroHeading));
    new JoystickButton(rightJoystick, 3).onTrue(new OverrideCmd(swerveSubsystem, intakeMotorSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
    new JoystickButton(rightJoystick, 13).onTrue(new InstantCommand(swerveSubsystem :: disableCams));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftBumper().onTrue(new InstantCommand(pnuematicsSubsystem :: toggleSmallpnuematics));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightBumper().onTrue(new InstantCommand(pnuematicsSubsystem :: toggleBigpnuematics));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).a().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorIntakePresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).b().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorSpeakerPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).x().onTrue(new SetArmPitchCmd(pitchMotorSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorAmpPresetAngle));

    new CommandXboxController(OperatorConstants.kXboxControllerPort).rightTrigger(0.5).onTrue(new runShooter(shootingMotorSubsystem, intakeMotorSubsystem, 0.9, 4000));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).leftTrigger(0.5).onTrue(new runShooter(shootingMotorSubsystem, intakeMotorSubsystem, 0.4, 0));
    //new CommandXboxController(OperatorConstants.kXboxControllerPort).y().onTrue(new SetArmPitchCmd(armSubsystem, ArmMotorsConstants.PitchMotor.kPitchMotorStandbyPresetAngle));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).start().onTrue( new FastAutoAimCmd(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem, intakeMotorSubsystem));
    new CommandXboxController(OperatorConstants.kXboxControllerPort).back().onTrue(new ConstantAimToggleCmd(swerveSubsystem, pitchMotorSubsystem, shootingMotorSubsystem));
  }

  public Command getAutonomousCommand() {
    System.out.println("Autos Begun");
       
      m_autoSelected = m_chooser.getSelected();

      if(m_autoSelected == SPtwoNtwoNone)
        return AutoBuilder.buildAuto(SPtwoNtwoNone);

      if (m_autoSelected == SPtwoNtwo)
        return AutoBuilder.buildAuto(SPtwoNtwo);

      if(m_autoSelected == SPtwoNoneNtwoNthree)
        return AutoBuilder.buildAuto(SPtwoNoneNtwoNthree);

      if(m_autoSelected == SPtwoNthreeNtwoNoneNfour)
        return AutoBuilder.buildAuto(SPtwoNthreeNtwoNoneNfour);
      
      if(m_autoSelected == SPtwoNoneNfour)
        return AutoBuilder.buildAuto(SPtwoNoneNfour);
      
      if(m_autoSelected == SPtwoNtwoNfour)
        return AutoBuilder.buildAuto(SPtwoNtwoNfour);

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

}
