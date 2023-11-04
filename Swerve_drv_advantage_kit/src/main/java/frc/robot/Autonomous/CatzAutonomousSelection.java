package frc.robot.Autonomous;

import java.security.DrbgParameters.Reseed;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
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



public class CatzAutonomousSelection 
{

    public final LoggedDashboardChooser<Enum> chosenAllianceColor = new LoggedDashboardChooser<>("/alliance selector");
    public final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("/Auto Routine");

    public CatzAutonomousCmds autonPaths = new CatzAutonomousCmds();
    public CatzAutonomousSelection()
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
        return new TrajectoryFollowingCmd(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(180.0));
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








    
    



}
