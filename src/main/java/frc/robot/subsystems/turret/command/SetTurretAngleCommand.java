package frc.robot.subsystems.turret.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretOI;
import frc.robot.subsystems.turret.Turret;


public class SetTurretAngleCommand extends CommandBase {
    private Turret turret;
    private TurretOI oi;
    public SetTurretAngleCommand(Turret turret, TurretOI oi) {
        this.turret = turret;
        this.oi = oi;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        turret.setTurretAngle(this.oi.getTurretAngle());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
