//package org.firstinspires.ftc.teamcode.autos
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import io.github.bionictigers.axiom.core.commands.bt.BtCommand
//import io.github.bionictigers.axiom.core.commands.bt.behaviorTree
//import io.github.bionictigers.axiom.core.commands.bt.composites.Parallel
//import io.github.bionictigers.axiom.core.commands.bt.composites.ParallelPolicy
//import io.github.bionictigers.axiom.core.commands.bt.composites.Sequence
//import io.github.bionictigers.axiom.core.commands.bt.leaves.BtAction
//import io.github.bionictigers.axiom.core.commands.bt.leaves.Wait
//import io.github.bionictigers.axiom.core.commands.bt.traced
//import io.github.bionictigers.axiom.core.scheduler.Scheduler
//import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
//import org.firstinspires.ftc.teamcode.mechanisms.Intake
//import org.firstinspires.ftc.teamcode.mechanisms.Limelight
//import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
//import org.firstinspires.ftc.teamcode.mechanisms.Odometry
//import org.firstinspires.ftc.teamcode.mechanisms.Persistents
//import org.firstinspires.ftc.teamcode.mechanisms.Shooter
//import org.firstinspires.ftc.teamcode.mechanisms.Transfer
//import org.firstinspires.ftc.teamcode.utils.Distance
//import org.firstinspires.ftc.teamcode.utils.Pose
//import kotlin.math.abs
//import kotlin.time.Duration.Companion.milliseconds
//
//
///*
//{
// "version": 1,
// "units": "mm",
// "fieldSize": {
//   "width": 3657.6,
//   "height": 3657.6
// },
// "startPose": {
//   "xMm": 1205.693952003203,
//   "yMm": 2742.2484920143665,
//   "headingDeg": -90
// },
// "waypoints": {
//   "Start": {
//     "xMm": 1543.659364311818,
//     "yMm": 3435.2183881959845,
//     "headingDeg": -90
//   },
//   "QueueClose": {
//     "xMm": 1205.693952003203,
//     "yMm": 2742.2484920143665,
//     "headingDeg": 180
//   },
//   "GrabClose": {
//     "xMm": 407.64117167781785,
//     "yMm": 2739.408451456245,
//     "headingDeg": -180
//   },
//   "QueueMiddle": {
//     "xMm": 1211.374042966373,
//     "yMm": 2125.959690902026,
//     "headingDeg": -90
//   },
//   "GrabMiddle": {
//     "xMm": 390.60089878830775,
//     "yMm": 2128.7997314601475,
//     "headingDeg": -90
//   },
//   "ShootClose": {
//     "xMm": 1830.5039579519032,
//     "yMm": 1796.5149861599455,
//     "headingDeg": -90
//   },
//   "QueueFar": {
//     "xMm": 989.8504954027429,
//     "yMm": 1509.6708897896856,
//     "headingDeg": -90
//   },
//   "GrabFar": {
//     "xMm": 384.9208078251378,
//     "yMm": 1523.8710925802923,
//     "headingDeg": -85
//   }
// },
// "moves": [
//   {
//     "xMm": 1205.693952003203,
//     "yMm": 2742.2484920143665,
//     "headingDeg": -170
//   },
//   {
//     "xMm": 407.64117167781785,
//     "yMm": 2739.408451456245,
//     "headingDeg": -180
//   },
//   {
//     "xMm": 1540.819318830233,
//     "yMm": 3432.378347637863,
//     "headingDeg": -180
//   },
//   {
//     "xMm": 1205.693952003203,
//     "yMm": 2742.2484920143665,
//     "headingDeg": 180
//   },
//   {
//     "xMm": 1543.659364311818,
//     "yMm": 3435.2183881959845,
//     "headingDeg": -90
//   },
//   {
//     "xMm": 1543.659364311818,
//     "yMm": 3429.5383070797416,
//     "headingDeg": -90
//   },
//   {
//     "xMm": 1211.374042966373,
//     "yMm": 2125.959690902026,
//     "headingDeg": 180
//   },
//   {
//     "xMm": 390.60089878830775,
//     "yMm": 2128.7997314601475,
//     "headingDeg": 20
//   },
//   {
//     "xMm": 1830.5039579519032,
//     "yMm": 1796.5149861599455,
//     "headingDeg": -80
//   },
//   {
//     "xMm": 989.8504954027429,
//     "yMm": 1509.6708897896856,
//     "headingDeg": -85
//   },
//   {
//     "xMm": 384.9208078251378,
//     "yMm": 1523.8710925802923,
//     "headingDeg": -85
//   },
//   {
//     "xMm": 1827.663912470318,
//     "yMm": 1790.8349050437025,
//     "headingDeg": -85
//   },
//   {
//     "xMm": 1543.659364311818,
//     "yMm": 3435.2183881959845,
//     "headingDeg": -90
//   },
//   {
//     "xMm": 1543.659364311818,
//     "yMm": 3435.2183881959845,
//     "headingDeg": -90
//   },
//   {
//     "xMm": 1205.693952003203,
//     "yMm": 2742.2484920143665,
//     "headingDeg": 180
//   },
//   {
//     "xMm": 1205.693952003203,
//     "yMm": 2742.2484920143665,
//     "headingDeg": 180
//   },
//   {
//     "xMm": 1205.693952003203,
//     "yMm": 2742.2484920143665,
//     "headingDeg": 180
//   },
//   {
//     "xMm": 407.64117167781785,
//     "yMm": 2739.408451456245,
//     "headingDeg": -180
//   },
//   {
//     "xMm": 390.60089878830775,
//     "yMm": 2128.7997314601475,
//     "headingDeg": -90
//   }
// ],
// "robotSize": {
//   "widthMm": 457.2,
//   "heightMm": 457.2
// }
//}
//*/
//
//
///*
//   Shoot Routine:
//       Move to start / close Pose
//      Spin up Shooter
//      Wait for velocity to reach shooting threshold
//      Feed the transfer into the shooter
//
//   Pickup Routine:
//       Move to queue pose
//       Set Transfer to Intake Positon
//       Spin intake
//       Drive at 30% speed
//       Whenever color sensor reads, move to next empty slot
//
//   Main Routine:
//
//*/
//@Autonomous
//class BlueFar : LinearOpMode() {
//    private val startPose = Pose(Distance.inch(9).mm, 609.6 * 3.5, 90)
//    private val farShootPose = Pose(Distance.inch(20).mm, 609.6 * 3.5, 60)
//    private val shootClosePose = Pose(609.6 * 3.5, 609.6 * 3.5, 90 - 45)
//    private val queueClosePose = Pose(609.6 * 1.5, 609.6 * 4, 0)
//    private val grabClosePose = Pose(609.6 * 1.5, 609.6 * 5.5, 0)
//    private val queueMiddlePose = Pose(1211.37, 2125.95, 0)
//    private val grabMiddlePose = Pose(390.60, 2128.79, 0)
//    private val queueFarPose = Pose(989.85, 1509.67, 0)
//    private val grabFarPose = Pose(384.92, 1523.87, 0)
//
//    override fun runOpMode() {
//        val octoQuad = OctoQuad(hardwareMap, telemetry)
//        val odometry = Odometry(hardwareMap, telemetry, startPose)
//        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry, octoQuad)
//        val intake = Intake(hardwareMap)
//        val limeLight = Limelight(hardwareMap, telemetry, false)
//        val transfer = Transfer(hardwareMap, octoQuad, telemetry)
//        val shooter = Shooter(hardwareMap, odometry, limeLight, telemetry, false)
//
//        Scheduler.telemetry = telemetry
//        Scheduler.schedule(octoQuad, odometry, drivetrain, intake, transfer, shooter)
//
//        val autoTree = behaviorTree("BlueFar Auto") {
//            sequence("Main Routine") {
//                add(
//                    shootRoutineFactory(
//                        name = "Preload Shoot",
//                        drivetrain = drivetrain,
//                        shooter = shooter,
//                        transfer = transfer,
//                        shootPose = farShootPose,
//                        useCloseVelocity = false
//                    )
//                )
//                add(
//                    cycleFactory(
//                        name = "Close Cycle",
//                        drivetrain = drivetrain,
//                        intake = intake,
//                        shooter = shooter,
//                        transfer = transfer,
//                        queuePose = queueClosePose,
//                        grabPose = grabClosePose,
//                        shootPose = shootClosePose,
//                        useCloseVelocity = true
//                    )
//                )
//                add(
//                    cycleFactory(
//                        name = "Middle Cycle",
//                        drivetrain = drivetrain,
//                        intake = intake,
//                        shooter = shooter,
//                        transfer = transfer,
//                        queuePose = queueMiddlePose,
//                        grabPose = grabMiddlePose,
//                        shootPose = shootClosePose,
//                        useCloseVelocity = true
//                    )
//                )
//                add(
//                    cycleFactory(
//                        name = "Far Cycle",
//                        drivetrain = drivetrain,
//                        intake = intake,
//                        shooter = shooter,
//                        transfer = transfer,
//                        queuePose = queueFarPose,
//                        grabPose = grabFarPose,
//                        shootPose = shootClosePose,
//                        useCloseVelocity = true
//                    )
//                )
//                add(moveToPoseFactory("Park", drivetrain, queueClosePose, speed = 1.0, timeoutMs = 3000))
//                add(
//                    BtAction("Shutdown") {
//                        stopShooter(shooter)
//                        Scheduler.schedule(intake.stop())
//                        succeed()
//                    }
//                )
//            }
//        }.traced()
//
//        waitForStart()
//        if (isStopRequested) {
//            Scheduler.clear()
//            Persistents.currentPose = odometry.position
//            return
//        }
//
//        Scheduler.schedule(autoTree)
//
//        while (opModeIsActive()) {
//            Scheduler.tick()
//            telemetry.addData("Auto Node Status", autoTree.treeStatus)
//            telemetry.addData("Transfer Occupied", transfer.occupiedBays)
//            telemetry.update()
//        }
//
//        Scheduler.clear()
//        Persistents.currentPose = odometry.position
//    }
//
//    private fun cycleFactory(
//        name: String,
//        drivetrain: Drivetrain,
//        intake: Intake,
//        shooter: Shooter,
//        transfer: Transfer,
//        queuePose: Pose,
//        grabPose: Pose,
//        shootPose: Pose,
//        useCloseVelocity: Boolean
//    ): BtCommand<*> {
//        return Sequence(
//            name,
//            listOf(
//                pickupRoutineFactory(
//                    name = "$name Pickup",
//                    drivetrain = drivetrain,
//                    intake = intake,
//                    shooter = shooter,
//                    transfer = transfer,
//                    queuePose = queuePose,
//                    grabPose = grabPose
//                ),
//                shootRoutineFactory(
//                    name = "$name Shoot",
//                    drivetrain = drivetrain,
//                    shooter = shooter,
//                    transfer = transfer,
//                    shootPose = shootPose,
//                    useCloseVelocity = useCloseVelocity
//                )
//            )
//        )
//    }
//
//    private fun pickupRoutineFactory(
//        name: String,
//        drivetrain: Drivetrain,
//        intake: Intake,
//        shooter: Shooter,
//        transfer: Transfer,
//        queuePose: Pose,
//        grabPose: Pose
//    ): BtCommand<*> {
//        return Sequence(
//            name,
//            listOf(
//                moveToPoseFactory("$name Queue Move", drivetrain, queuePose, speed = 0.5, timeoutMs = 2600),
//                BtAction("$name Intake Setup") {
//                    stopShooter(shooter)
//                    Scheduler.schedule(intake.intake())
//                    succeed()
//                },
//                moveToPoseFactory("$name Grab Move", drivetrain, grabPose, speed = 0.35, timeoutMs = 3200),
//                intakeFillRoutineFactory(
//                    name = "$name Fill Transfer",
//                    transfer = transfer,
//                    targetOccupiedBays = 3,
//                    timeoutMs = 2200
//                ),
//                BtAction("$name Intake Stop") {
//                    Scheduler.schedule(intake.stop())
//                    succeed()
//                }
//            )
//        )
//    }
//
//    private fun shootRoutineFactory(
//        name: String,
//        drivetrain: Drivetrain,
//        shooter: Shooter,
//        transfer: Transfer,
//        shootPose: Pose,
//        useCloseVelocity: Boolean
//    ): BtCommand<*> {
//        val velocityTolerance = if (useCloseVelocity) 80.0 else 60.0
//        return Sequence(
//            name,
//            listOf(
//                Parallel(
//                    "$name Shooter Move",
//                    children = listOf(
//                        moveToPoseFactory(
//                            "$name Drive",
//                            drivetrain,
//                            shootPose,
//                            speed = 1.0,
//                            timeoutMs = 3200
//                        ),
//                        BtAction("$name Shooter Setup") {
//                            startShooter(shooter)
//                            succeed()
//                        }
//                    ),
//                    ParallelPolicy.REQUIRE_ONE
//                ),
//                Wait("$name Shooter Settle", 3000.milliseconds),
//                waitForShooterVelocityFactory(
//                    "$name Velocity Wait",
//                    shooter,
//                    tolerance = velocityTolerance,
//                    timeoutMs = 1500
//                ),
//                BtAction("$name Feed Transfer") {
//                    Scheduler.schedule(transfer.shoot())
//                    succeed()
//                },
//                Wait("$name Transfer Feed", transfer.shootingTime + 250.milliseconds),
//                BtAction("$name Shooter Stop") {
//                    stopShooter(shooter)
//                    succeed()
//                }
//            )
//        )
//    }
//
//    private fun startShooter(shooter: Shooter) {
//        if (!shooter.isActive) {
//            Scheduler.schedule(shooter.start())
//        }
//    }
//
//    private fun stopShooter(shooter: Shooter) {
//        if (shooter.isActive) {
//            Scheduler.schedule(shooter.stop())
//        }
//    }
//
//    private fun moveToPoseFactory(
//        name: String,
//        drivetrain: Drivetrain,
//        targetPose: Pose,
//        speed: Double,
//        timeoutMs: Long
//    ): BtCommand<*> {
//        var started = false
//        return BtAction(name) {
//            if (!started) {
//                started = true
//                Scheduler.schedule(drivetrain.moveToPosition(targetPose, speed))
//                // Skip checking mtp on the first tick — the moveToPosition
//                // command's enter{} hasn't run yet so mtp is still null.
//                return@BtAction
//            }
//
//            if (drivetrain.mtp == null) {
//                println("move success")
//                succeed()
//            }
//
//            if (meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds) {
//                println("move failed")
////                drivetrain.mtp = null
//                succeed()
//            }
//        }
//    }
//
//    private fun waitForShooterVelocityFactory(
//        name: String,
//        shooter: Shooter,
//        tolerance: Double,
//        timeoutMs: Long
//    ): BtCommand<*> {
//        return BtAction(name) {
//            val targetVelocity = shooter.targetVelocity
//            // A 100000 target means "just give the flywheel time to spin up".
//            val ready = targetVelocity >= 100000.0 ||
//                targetVelocity <= 0.0 ||
//                abs(shooter.currentVelocity - targetVelocity) <= tolerance
//            val timedOut = meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds
//            if (ready || timedOut) {
//                succeed()
//            }
//        }
//    }
//
//    private fun intakeFillRoutineFactory(
//        name: String,
//        transfer: Transfer,
//        targetOccupiedBays: Int,
//        timeoutMs: Long
//    ): BtCommand<*> {
//        return BtAction(name) {
//            if (transfer.occupiedBays >= targetOccupiedBays || meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds) {
//                succeed()
//            }
//        }
//    }
//}