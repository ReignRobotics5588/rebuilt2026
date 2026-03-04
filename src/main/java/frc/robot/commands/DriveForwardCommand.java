package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCommand extends Command {
    private final DriveSubsystem m_drive;
    private final double m_distanceMeters;
    private final double m_speed;
    private double[] m_startXY = new double[2];

    public DriveForwardCommand(DriveSubsystem drive, double distanceMeters, double speed) {
        m_drive = drive;
        m_distanceMeters = Math.abs(distanceMeters);
        m_speed = Math.abs(speed)*(distanceMeters < 0 ? -1.0 : 1.0);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        var p = m_drive.getPose();
        m_startXY[0] = p.getX();
        m_startXY[1] = p.getY();
    }

    @Override
    public void execute(){
        m_drive.drive(m_speed, 0.0, 0.0, false);
        // drive to X (forward/back) direction
    }

    @Override
    public void end (boolean interrupted) {
       m_drive.drive(0.0, 0.0, 0.0, false);
    }
    
    @Override
    public boolean isFinished() {
        var p = m_drive.getPose();
        double dx = p.getX() - m_startXY[0];
        double dy = p.getY() - m_startXY[1];
        double dist = Math.hypot(dx, dy);
        return dist >= m_distanceMeters;

    }
// do I need to add this to the AutoCommandFactory? Obviously after I fix all the glaring errors
    
}
