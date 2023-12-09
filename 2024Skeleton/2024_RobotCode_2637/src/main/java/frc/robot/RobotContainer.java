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

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.Commands.TeleopDriveCmd;
import frc.robot.Subsystems.drivetrain.SubsystemCatzDriveTrain;
import frc.robot.Utils.CatzAbstractStateUtil;
import frc.robot.Utils.CatzAbstractStateUtil.GamePieceState;
import frc.robot.Utils.CatzAbstractStateUtil.SetAbstractMechanismState;
import frc.robot.Utils.led.CatzRGB;
import frc.robot.Utils.led.ColorMethod;


 
 
 /**
  * This class is where the bulk of the robot should be declared. Since Command-based is a
  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
  * subsystems, commands, and trigger mappings) should be declared here.
  */
 public class RobotContainer {
    
    //subsystems
    private SubsystemCatzDriveTrain driveTrain; 

    //private final CatzAutonomous auton = new CatzAutonomous();
    private static CatzRGB        led = new CatzRGB();

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
   public RobotContainer() {
    //instantiate subsystems
     driveTrain = SubsystemCatzDriveTrain.getInstance(); 


 
     xboxDrv = new CommandXboxController(XBOX_DRV_PORT); 
     xboxAux = new CommandXboxController(XBOX_AUX_PORT);
 
 
     // Configure the trigger bindings and default cmds
     defaultCommands();
     configureBindings();
   }
 
   
   private void configureBindings() {


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
     return new InstantCommand();
   }

   //--------------------------------------------LED setup------------------------------------------------
   //TBD in constants class
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

 }