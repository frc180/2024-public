### DriveSubsystem
| Device Name | Device Type | CAN ID | Breaker |
| ------ | ------ | ------ | ------ |
| driveMotorFL | TalonFX - Kraken | 10 | # |
| turningMotorFL | TalonFX - Kraken | 11 | # |
| turningAbsoluteFL | CANCoder | 3 | # |
| driveMotorFR | TalonFX - Kraken | 12 | # |
| turningMotorFR | TalonFX - Kraken | 13 | # |
| turningAbsoluteFR | CANCoder | 4 | # |
| driveMotorRL | TalonFX - Kraken | 14 | # |
| turningMotorRL | TalonFX - Kraken | 15 | # |
| turningAbsoluteRL | CANCoder | 5 | # |
| driveMotorRR | TalonFX - Kraken | 16 | # |
| turningMotorRR | TalonFX - Kraken | 17 | # |
| turningAbsoluteRR | CANCoder | 6 | # |
| pigeonIMU | Pigeon IMU | 7 | # | 

### ShooterSubsystem
| Device Name | Device Type | CAN ID | Breaker |
| ------ | ------ | ------ | ------ |
| shooterMotor1 | Spark Flex - Vortex | 20 | # |
| shooterMoter2 | Spark Flex - Vortex | 21 | # |
| shooterPivotMotor | Spark Max - NEO 550 | 22 | # |

### IntakeSubsystem
| Device Name | Device Type | CAN ID | Breaker |
| ------ | ------ | ------ | ------ |
| intakeMotor | TalonFX - Kraken | 23 | # |
| sweeperMotor | Spark Max - NEO 550 | 24 | # |
| backIntakeMotor | TalonFX - Kraken | 32 | # |

### IntakePivotSubsystem
| Device Name | Device Type | CAN ID | Breaker |
| ------ | ------ | ------ | ------ |
| intakePivotMotor | Spark Max - NEO 550 | 25 | # |

### WinchSubsystem
| Device Name | Device Type | CAN ID | Breaker |
| ------ | ------ | ------ | ------ |
| winchMotor1 | Spark Max - NEO | 27 | # |
| winchMotor2 | Spark Max - NEO | 28 | # |

### AmpSubsystem
| Device Name | Device Type | CAN ID | Breaker |
| ------ | ------ | ------ | ------ |
| ampArmMotor | Spark Max - NEO 550 | 29 | # |
| ampRollerMotor | Spark Max - NEO 550 | 30 | # |

### FanSubsystem
| Device Name | Device Type | CAN ID | Breaker |
| ------ | ------ | ------ | ------ |
| fanMotor | Spark Max - 775 Redline | 31 | # |

Robot code for our 2024 FRC season.

Libraries/APIs used in this project:

* [Phoenix 6](https://pro.docs.ctr-electronics.com/en/latest/) - Used for controlling CTRE products (i.e. Krakens, TalonFX, CANCoders)
* [REVLib](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information) - Used for controlling REV Robotics products (i.e. Sparks & NEOs)
* [PathPlannerLib](https://pathplanner.dev/pplib-getting-started.html) - Used for following paths created by [PathPlanner](https://pathplanner.dev/home.html) and [Choreo](https://sleipnirgroup.github.io/Choreo/)
* [ChoreoLib](https://sleipnirgroup.github.io/Choreo/choreolib/usage/) - Used for following paths created by [Choreo](https://sleipnirgroup.github.io/Choreo/)
* [Limelight](https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api) - Used for targeting April Tags and Notes.
* [LimelightLib](https://github.com/LimelightVision/limelightlib-wpijava) - Used to make reading Limelight data easier
* [Monologue](https://github.com/shueja/Monologue/wiki) - Used for logging data to NetworkTables and WPILib log files for later viewing
