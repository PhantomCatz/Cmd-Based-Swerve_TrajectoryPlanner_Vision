package frc.robot.Autonomous;

import java.nio.file.Path;
import java.security.DrbgParameters.Reseed;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.Autonomous.Trajectory.TrajectoryFollowingCmd;
import frc.robot.Autonomous.Trajectory.Paths.Trajectories;
import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.SetMechanismState;
import frc.robot.commands.StateMachineCmd;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class CatzAutonomous 
{
    private final static CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
    private final static CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
    private final static CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
    private final static CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();

    //private static PathPlannerPath driveStraighFullTurn = PathPlannerPath.fromPathFile("DriveStraightFullTurn");
    //private static PathPlannerPath feildSideDriveBack = PathPlannerPath.fromPathFile("FeildSideDriveBack");


    public LoggedDashboardChooser<Enum> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    public LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    public CatzAutonomous()
    {
        chosenAllianceColor.addDefaultOption("Blue Alliance", CatzConstants.AllianceColor.BlUE_ALLIANCE);
        chosenAllianceColor.addOption       ("Red Alliance",  CatzConstants.AllianceColor.RED_ALLIANCE);

        autoChooser.addDefaultOption("Do Nothing", null);

        autoChooser.addOption       ("TEST PATH",  testPath());
        autoChooser.addOption       ("start from wall", startWall());
        autoChooser.addOption       ("start from feild", startField());
        autoChooser.addOption       ("start from Center", startField());
    }

    public Command testPath()
    {
        return new SequentialCommandGroup(

               new TrajectoryFollowingCmd(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(180))
                                        );

    }

    public Command startWall()
    {
        return new SequentialCommandGroup();
    }

    public Command startField()
    {
        return new SequentialCommandGroup();
    }

    public Command startCenter()
    {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup parallelScoreCube()
    {
        return new SequentialCommandGroup(
                    Commands.runOnce(() -> CatzStateUtil.newGamePieceState(GamePieceState.CUBE)),
                    new StateMachineCmd(ManipulatorPoseConstants.SCORE_HIGH_CONE)
                        .raceWith(Commands.waitSeconds(1.0)),

                    new ParallelCommandGroup(
                       // new PPTrajectoryFollowingCmd(driveStraighFullTurn), 
                        new SequentialCommandGroup(
                            new StateMachineCmd(ManipulatorPoseConstants.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                           // new StateMachineCmd(SetMechanismState.PICKUP_GROUND)
                          //      .raceWith(Commands.waitSeconds(1.0)),
                            intakeSequenceCommandGroup())
                                            ),

                    new ParallelCommandGroup(
                       // new PPTrajectoryFollowingCmd(feildSideDriveBack), 
                        new SequentialCommandGroup(
                            new StateMachineCmd(ManipulatorPoseConstants.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                            new StateMachineCmd(ManipulatorPoseConstants.SCORE_HIGH_CONE)
                                .raceWith(Commands.waitSeconds(1.0)))
                                            ),
                    intakeSequenceCommandGroup(),
                    new StateMachineCmd(ManipulatorPoseConstants.STOW)
                                         );       
    }

    public static SequentialCommandGroup RightScore1HighConeBalance()
    {
        return new SequentialCommandGroup(
                    Commands.runOnce(() -> CatzStateUtil.newGamePieceState(GamePieceState.CONE)),
                    new StateMachineCmd(CatzConstants.ManipulatorPoseConstants.SCORE_HIGH_CONE)

                   //new PPTrajectoryFollowingCmd(driveStraighFullTurn)
            
        );
    }

    public static SequentialCommandGroup intakeSequenceCommandGroup()
    {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> intake.intakeRollerFunctionIN()),
                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> intake.intakeRollersOff())           
                                         );
    }








    
    



}
