package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.GameModeLED;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

/***
 * Robot.java
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * Robot.java now only serves to run the command scheduler every 20 ms in robot periodic
 * -This single loop runs all periodic functions gaurnteeing the accuracy of loops when
 * replaying in simulator.
 ***/

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();

    // Record metadata
    // logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    // logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    // logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    // logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    // logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    // switch (BuildConstants.DIRTY) {
    //   case 0:
    //     logger.recordMetadata("GitDirty", "All changes committed");
    //     break;
    //   case 1:
    //     logger.recordMetadata("GitDirty", "Uncomitted changes");
    //     break;
    //   default:
    //     logger.recordMetadata("GitDirty", "Unknown");
    //     break;
    // }

    // Set up data receivers & replay source
    switch (CatzConstants.currentMode) {
      // Running on a real robot, log to a USB stick
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        logger.addDataReceiver(new NT4Publisher());
       // new PowerDistribution(1, ModuleType.kRev);
        break;

      // Running a physics simulator, log to local folder
      case SIM:
        logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
        logger.addDataReceiver(new NT4Publisher());
        break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }
    // Start AdvantageKit logger
    logger.start();
  
    // Instantiate our RobotContainer. This will perform all our button mappings to triggers, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    DriverStation.silenceJoystickConnectionWarning(true);
  }


  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    //Logging scheudled commands
    Logger.getInstance().recordOutput("ActiveCommands/Scheduler", 
          NetworkTableInstance.getDefault()
                              .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
                              .getStringArray(new String[] {}));
                            
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {
    RobotContainer.currentGameModeLED = GameModeLED.MatchEnd;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.currentGameModeLED = GameModeLED.InAutonomous;
  }

  @Override
  public void autonomousPeriodic() { }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.currentGameModeLED = GameModeLED.TeleOp;
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
