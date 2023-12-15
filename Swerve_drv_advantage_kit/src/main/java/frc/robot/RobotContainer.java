package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.Utils.CatzAbstractStateUtil;
import frc.robot.Utils.CatzAbstractStateUtil.GamePieceState;
import frc.robot.Utils.CatzAbstractStateUtil.SetAbstractMechanismState;
import frc.robot.Utils.led.CatzRGB;
import frc.robot.Utils.led.ColorMethod;
import frc.robot.commands.ManipulatorToPoseCmd;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.commands.ManualStateCmds.ArmManualCmd;
import frc.robot.commands.ManualStateCmds.ElevatorManualCmd;
import frc.robot.commands.ManualStateCmds.IntakeManualCmd;
import frc.robot.subsystems.Arm.SubsystemCatzArm;
import frc.robot.subsystems.Elevator.SubsystemCatzElevator;
import frc.robot.subsystems.Intake.SubsystemCatzIntake;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;

/**
 * RobotContainer
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is how the command scheduler(robot.java replacment) is configured
 * Configures:
 * -xbox controller triggers
 * -default commands
 * -instanciated mechanisms using singleton implementation
 * -sets up autonomous from CatzAtutonomouschooser
 */
 public class RobotContainer {
    
    //subsystems
    private CatzDriveTrainSubsystem driveTrain; 
    private SubsystemCatzElevator elevator;
    private SubsystemCatzIntake intake;
    private SubsystemCatzArm arm;

    private final CatzAutonomous auton = new CatzAutonomous();
    private static CatzRGB        led = new CatzRGB();

    //xbox controller
    private CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;

    //RobotContainer Constants
    private final int XBOX_DRV_PORT = 0;
    private final int XBOX_AUX_PORT = 1;
 
       
   /** The container for the robot. Contains subsystems, OI devices, and commands. 
    *    -since multiple classes are referencing these mechansims, 
    *     mechanisms are instantiated inside mechanism class(singleton)
   */
   public RobotContainer() {
    //instantiate subsystems
     driveTrain = CatzDriveTrainSubsystem.getInstance(); 
     elevator = SubsystemCatzElevator    .getInstance();
     arm = SubsystemCatzArm              .getInstance();          
     intake = SubsystemCatzIntake        .getInstance();
 
     xboxDrv = new CommandXboxController(XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(XBOX_AUX_PORT);
 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
   
   private void configureBindings() {
   //---------------------------------------Set State mechanism cmds-----------------------------------------------------------------
     xboxAux.y().onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.SCORE_HIGH)); //TBD when is "new" used? when you want to start up new object command instead of inline
     xboxAux.b().onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.SCORE_MID));
     xboxAux.a().onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.SCORE_LOW));
     xboxAux.x().or(xboxDrv.rightStick())
                .onTrue(new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW));
     xboxAux.start().or(xboxDrv.leftStick())
                    .onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.PICKUP_GROUND));
 
   
   //--------------------------------------------Manual Cmds---------------------------------------------------------------------------
     //arm
     xboxAux.rightTrigger().onTrue(new ArmManualCmd(true))
                           .onFalse(arm.setArmPwrCmd(0.0));
 
     xboxAux.leftTrigger().onTrue(new ArmManualCmd(false))
                          .onFalse(arm.setArmPwrCmd(0.0));
    //intake
     xboxAux.leftStick().onTrue(new IntakeManualCmd(()-> xboxAux.getLeftY(), 
                                                   ()-> xboxAux.leftStick().getAsBoolean()));
    //elevator     
     xboxAux.rightStick().onTrue(new ElevatorManualCmd(()-> xboxAux.getRightY(), 
                                                       ()-> xboxAux.rightStick().getAsBoolean()));
  
 
 
   //-----------------------------------commands with no subsystem--------------------------------------------------------------------
     xboxAux.back()
        .onTrue(Commands.runOnce(() -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.NONE)));
 
     xboxAux.povLeft()
     .onTrue(Commands.runOnce(
         () -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.CUBE)
                             ));
 
     xboxAux.povRight()
     .onTrue(Commands.runOnce(
        () -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.CONE)
                             ));
 
     //disabling softlimits only when both bumpers are pressed
     //TBD how to set precedence with and and or methods
     Trigger softLimitTrigger = xboxAux.leftBumper().and(xboxAux.rightBumper()); //tbd which is evaluated first? First one
     softLimitTrigger.onTrue(intake.softLimitOverideDisabled());
     softLimitTrigger.onFalse(intake.softLimitOverideEnabled());
 
 
     xboxDrv.start().onTrue(driveTrain.zeroGyro());
 
     xboxDrv.b().onTrue(driveTrain.stopDriving()); //TBD need to add this back in TBD runs when disabled where?
 
     
     //--------------------------Intake Rollers--------------------------
       xboxAux.rightBumper().onTrue(intake.intakeRollersIn())
                            .onFalse(intake.intakeRollersOff());//tbd look more ar inline commands and look at the different types of methods
                             
       xboxAux.leftBumper().onTrue(intake.intakeRollersOut())
                            .onFalse(intake.intakeRollersOff());

   }
   //TBD
   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() {  
      driveTrain.setDefaultCommand(new TeleopDriveCmd(()-> xboxDrv.getLeftX(),
                                                      ()-> xboxDrv.getLeftY(),
                                                      ()-> xboxDrv.getRightX(),
                                                      ()-> xboxDrv.getRightTriggerAxis(), 
                                                      ()-> xboxDrv.b().getAsBoolean()));
    
   }

   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
     return auton.getCommand();
   }

   //--------------------------------------------LED setup------------------------------------------------
   //TBD in constants class
   public static enum MechMode {
    AutoMode(Color.kGreen),
    ManualHoldMode(Color.kCyan),
    ManualMode(Color.kRed);

    public Color color;
    MechMode(Color color){
      this.color = color;
    }
  }

  public static enum GamePiece{
    Cube(Color.kPurple),
    Cone(Color.kYellow),
    None(Color.kGhostWhite);

    public Color color;
    GamePiece(Color color){
      this.color = color;
    }
  }

  public static enum GameModeLED{
    Autobalancing(led.oneColorFill, Color.kGreen),
    InAutonomous(led.startFlowing, led.PHANTOM_SAPPHIRE, Color.kWhite),
    MatchEnd(led.startFlowingRainbow),
    EndgameWheelLock(led.oneColorFillAllianceColor), 
    TeleOp(led.doNothing);

    public ColorMethod method;
    public Color[] color;
    private GameModeLED(ColorMethod method, Color... color) {
      this.method = method;
      this.color = color;
    }
  }

  public static MechMode intakeControlMode = MechMode.AutoMode;
  public static MechMode elevatorControlMode = MechMode.AutoMode;
  public static MechMode armControlMode = MechMode.AutoMode;
  public static GameModeLED currentGameModeLED = GameModeLED.MatchEnd;
  public static GamePiece currentGamePiece = GamePiece.None;

 }