package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.LiftPosition;

@Config
public class Lift extends SubsystemBase {
    public static double KP = .03;
    public static double KI = 0.008;
    public static double KD = 0.000;
    public static double KF = 0.000;
    public static double TE = 5.0;
    public static double TD = 5.0;

    private Level level = Level.Floor;
    private LiftPosition offset = new LiftPosition(0);
    private final MotorGroup motor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final PIDFController pidf;
    private boolean emergencyStop = false;
    private final LiftPosition MAX_HEIGHT;

    /**
     * Constructs a Lift with a HardwareMap.
     * @param hwMap the HardwareMap
     */
    public Lift(HardwareMap hwMap) {
        motor = new MotorGroup(
                new MotorEx(hwMap, "lift0", Motor.GoBILDA.RPM_435),
                new MotorEx(hwMap, "lift1", Motor.GoBILDA.RPM_435)
        );
        motor.setInverted(true);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(KP, KI, KD);
        motor.resetEncoder();
        pidf = new PIDFController(KP, KI, KD, KF);
        pidf.setTolerance(TE, TD);
        pidf.setSetPoint(0);
        MAX_HEIGHT = new LiftPosition(960);
    }

    private LiftPosition positionAvg() {
        return new LiftPosition(
                LiftPosition.ticksToMm(motor
                        .getPositions()
                        .stream()
                        .mapToDouble(d -> d)
                        .average()
                        .orElse(0.0)
                )
        );
    }

    /* **** Commands: **** */

    /**
     * Should be called in a loop to set the motor power.
     */
    public void update() {
        sendTelemetry();
        double speedTarget= pidf.calculate(positionAvg().ticks(), level.pos.ticks());
        if(emergencyStop || pidf.atSetPoint())
            motor.stopMotor();
        else
            motor.set(speedTarget);
        dashboardTelemetry.addData("PIDF calc target", speedTarget);

    }

    public void setOffset(double mm) {
        offset.setMm(mm);
    }
    public double getOffset() {
        return offset.mm();
    }

    /**
     * Moves the lift up 1 level (saturating).
     */
    public void up() {
        setLevel(level.up());
    }

    /**
     * Moves the lift down 1 level (saturating).
     */
    public void down() {
        setLevel(level.down());
    }

    /**
     * Gets the target level.
     * @return the current target level
     */
    public Level getLevel() {
        return level;
    }

    /**
     * An "emergency stop" if tuning makes the lift go crazy
     * Toggle forces the lift to 0 power
     */
    public void toggleStop(){
        emergencyStop = !emergencyStop;
    }

    /**
     * Set the height of the lift.
     * @param pos the position to set
     */
    public void setHeight(@NonNull LiftPosition pos) {
        pidf.setPIDF(KP, KI, KD, KF);
        pidf.setTolerance(TE, TD);
        //Range.clip protects from inadvertently setting it too high or too low
        pidf.setSetPoint(Range.clip(pos.ticks() + offset.ticks(),0, MAX_HEIGHT.ticks()));
    }

    public double getTargetHeight() {
        return offset.mm() + level.pos.mm();
    }

    /**
     * Sets the target level of the lift.
     * @param level the new level to set
     */
    public void setLevel(@NonNull Level level) {
        this.level = level;
        setHeight(level.pos);
    }

    /**
     * Sends telemetry data for tuning/debugging purposes. Can be graphed with FTC Dashboard
     * which is pretty nifty
     * The FTC Dashboard address is 192.168.43.1:8080/dash
     */
    public void sendTelemetry(){
        dashboardTelemetry.addData("Lift Position (mm)", positionAvg().mm());
        dashboardTelemetry.addData("Lift Target (mm)", level.pos.mm());
        dashboardTelemetry.addData("Lift Error(mm)", positionAvg().mm()-level.pos.mm());
        dashboardTelemetry.addData("Lift Velocity", motor.getVelocity()); //Ticks/second?
        dashboardTelemetry.update();
    }

    public enum Level {
        Floor(0.0),
        Short(250.0), //TODO: tune these to actual positions, in MILLIMETERS
        Medium(500.0),
        Long(960.0);

        public final LiftPosition pos;

        Level(double mmPos) {
            this.pos = new LiftPosition(mmPos);
        }

        public Level up() {
            switch (this) {
                case Floor:
                    return Short;
                case Short:
                    return Medium;
                case Medium:
                case Long:
                    return Long;
                default: // unreachable
                    return Floor;
            }
        }

        public Level down() {
            switch (this) {
                case Floor:
                case Short:
                    return Floor;
                case Medium:
                    return Short;
                case Long:
                    return Medium;
                default: // unreachable
                    return Long;
            }
        }
    }
}

