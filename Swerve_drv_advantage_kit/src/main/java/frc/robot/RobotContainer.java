/***
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
 ***/

 package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.BalanceCmd;
import frc.robot.Autonomous.CatzAutonomous;
import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzAbstractStateUtil;
 import frc.robot.Utils.CatzAbstractStateUtil.GamePieceState;
 import frc.robot.Utils.CatzAbstractStateUtil.SetAbstractMechanismState;
import frc.robot.Utils.led.CatzRGB;
import frc.robot.Utils.led.ColorMethod;
import frc.robot.commands.ManipulatorToPoseCmd;
import frc.robot.commands.TeleopDriveCmd;
import frc.robot.commands.ManualStateCmds.ArmManualCmd;
import frc.robot.commands.ManualStateCmds.ElevatorManualCmd;
import frc.robot.commands.ManualStateCmds.IntakeManualCmd;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
 import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
 import frc.robot.subsystems.Intake.CatzIntakeSubsystem;
 import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;
 
 
 /**
  * This class is where the bulk of the robot should be declared. Since Command-based is a
  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
  * subsystems, commands, and trigger mappings) should be declared here.
  */
 public class RobotContainer {
    private static CatzDriveTrainSubsystem driveTrain;
    private CatzElevatorSubsystem elevator;
    private CatzIntakeSubsystem intake;
    private CatzArmSubsystem arm;

    private final CatzAutonomous auton = new CatzAutonomous();
    public static CatzRGB        led = new CatzRGB();

    //xbox controller
    private CommandXboxController xboxDrv;
    private CommandXboxController xboxAux;

    //RobotContainer Constants
    private final int XBOX_DRV_PORT = 0;
    private final int XBOX_AUX_PORT = 1;
 
       
   /** The container for the robot. Contains subsystems, OI devices, and commands. 
    *    -since multiple classes are referencing these mechansims, 
    *         mechanisms are instantiated inside mechanism class(singleton)
   */
   public RobotContainer() 
   {
    //instantiate subsystems
     driveTrain = CatzDriveTrainSubsystem.getInstance(); 
     elevator = CatzElevatorSubsystem.getInstance();
     arm = CatzArmSubsystem.getInstance();
     intake = CatzIntakeSubsystem.getInstance();

 
     xboxDrv = new CommandXboxController(XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(XBOX_AUX_PORT);
 
 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
    //start led scheduling thread
    startLEDSchedulingThread();
   }
 
   
   private void configureBindings() 
   {
   //---------------------------------------Button mechanism cmds-----------------------------------------------------------------
     xboxAux.y().onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.SCORE_HIGH));
     xboxAux.b().onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.SCORE_MID));
     xboxAux.a().onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.SCORE_LOW));
     xboxAux.x().or(xboxDrv.rightStick())
                .onTrue(new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW));
     xboxAux.start().or(xboxDrv.leftStick())
                    .onTrue(new ManipulatorToPoseCmd(SetAbstractMechanismState.PICKUP_GROUND));
 
   
   //--------------------------------------------Manual Cmds---------------------------------------------------------------------------
     //arm
       
     xboxAux.rightTrigger()
     .onTrue(new ArmManualCmd(true,
                              false))
     .onFalse(Commands.runOnce(
             () -> arm.setArmPwr(0.0)
                              ));
 
     xboxAux.leftTrigger()
     .onTrue(new ArmManualCmd(false,
                              true))
     .onFalse(Commands.run(
             () -> arm.setArmPwr(0.0)
                          ));

    //intake
    xboxAux.leftStick().onTrue(new IntakeManualCmd(() -> xboxAux.getLeftY(), 
                                                   () -> xboxAux.leftStick().getAsBoolean()));
    //elevator     
     xboxAux.rightStick().onTrue(new ElevatorManualCmd(() -> xboxAux.getRightY(), 
                                                       () -> xboxAux.rightStick().getAsBoolean()));
  
 
 
   //-----------------------------------commands with no subsystem--------------------------------------------------------------------
     xboxAux.back()
     .onTrue(Commands.runOnce(
         () -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.NONE)
                             ));
 
     xboxAux.povLeft()
     .onTrue(Commands.runOnce(
         () -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.CUBE)
                             ));
 
     xboxAux.povRight()
     .onTrue(Commands.runOnce(
        () -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.CONE)
                             ));
 
      //disabling softlimits only when both bumpers are pressed
      
     xboxAux.leftBumper().and(xboxAux.rightBumper()) 
     .onTrue(Commands.runOnce(() -> intake.softLimitOverideDisabled()))
     .onFalse(Commands.runOnce(() -> intake.softLimitOverideEnabled()));
 
 
     xboxDrv.start().onTrue(Commands.runOnce(() -> driveTrain.zeroGyro()));
 
     //xboxDrv.b().onTrue(Commands.runOnce(() -> driveTrain.lockWheels())); TBD need to add this back in
 
     
     //--------------------------Intake Rollers--------------------------
       xboxAux.rightBumper().onTrue(Commands.runOnce(() -> intake.intakeRollerFunctionIN()))
                            .onFalse(Commands.runOnce(() -> intake.intakeRollersOff()));
 
       xboxAux.leftBumper().onTrue(Commands.runOnce(
         () -> {
           intake.intakeRollerFunctionOUT();
         })).onFalse(Commands.runOnce(
           () -> {
             intake.intakeRollersOff();
           }));
   }
   //mechanisms with default commands revert back to these cmds if no other cmd requiring the subsystem is active
   private void defaultCommands() 
   {  
      driveTrain.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(),
                                                      () -> xboxDrv.getLeftY(),
                                                      () -> xboxDrv.getRightX(),
                                                      () -> xboxDrv.getRightTriggerAxis(), 
                                                      () -> xboxDrv.b().getAsBoolean()));
                                                     
 
   }
   /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
   public Command getAutonomousCommand() {
     // An example command will be run in autonomous
     return auton.getCommand();
   }

   //--------------------------------------------Subsystem Get Methods-----------------------

   //--------------------------------------------LEDs------------------------------------------------
   public static enum mechMode {
    AutoMode(Color.kGreen),
    ManualHoldMode(Color.kCyan),
    ManualMode(Color.kRed);

    public Color color;
    mechMode(Color color){
      this.color = color;
    }
  }

  public static enum gamePiece{
    Cube(Color.kPurple),
    Cone(Color.kYellow),
    None(Color.kGhostWhite);

    public Color color;
    gamePiece(Color color){
      this.color = color;
    }
  }

  public static enum gameModeLED{
    Autobalancing(led.oneColorFill, Color.kGreen),
    InAutonomous(led.startFlowing, led.PHANTOM_SAPPHIRE, Color.kWhite),
    MatchEnd(led.startFlowingRainbow),
    EndgameWheelLock(led.oneColorFillAllianceColor), 
    TeleOp(led.doNothing);

    public ColorMethod method;
    public Color[] color;
    private gameModeLED(ColorMethod method, Color... color) {
      this.method = method;
      this.color = color;
    }
  }

  public static mechMode intakeControlMode = mechMode.AutoMode;
  public static mechMode elevatorControlMode = mechMode.AutoMode;
  public static mechMode armControlMode = mechMode.AutoMode;
  public static gameModeLED currentGameModeLED = gameModeLED.MatchEnd;
  public static gamePiece currentGamePiece = gamePiece.None;

   public void startLEDSchedulingThread() {
       Thread LEDThread = new Thread(() -> {

          //driverstation mode leds
          if(DriverStation.isAutonomous()) {
            currentGameModeLED = gameModeLED.InAutonomous;
          }
          else if(DriverStation.isTeleop()) {
            currentGameModeLED = gameModeLED.TeleOp;
          }

          //gamepiece mode leds
          if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
            currentGamePiece = gamePiece.Cone;
          }
          else if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CUBE) {
            currentGamePiece = gamePiece.Cube;
          }
          else {
            currentGamePiece = gamePiece.None;
          }

          
          Timer.delay(0.02);
       });
       LEDThread.start();
   }

 }