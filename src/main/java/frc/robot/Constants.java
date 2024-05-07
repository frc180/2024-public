package frc.robot;

public final class Constants {

    /**
     * The RPM to run the shooter at to be ready to make shots. Should be the 
     * highest RPM a shot can be taken at.
     */
    public static final double SHOOTER_IDLE = 4100;

    public static final double SHOOTER_IDLE_LOW = 2200;

    /**
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.546;

    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.546;

    /**
     * The time that elapses between each control loop.
     */
    public static final double LOOP_TIME = 0.020;

    /**
     * The width of the field in meters. Taken from ChoreoTrajectoryState.
     */
    public static final double FIELD_WIDTH_METERS = 16.5410515;
}
