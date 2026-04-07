//package org.firstinspires.ftc.teamcode.autos
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import io.github.bionictigers.axiom.core.commands.Command
//import io.github.bionictigers.axiom.core.commands.bt.BtCommand
//import io.github.bionictigers.axiom.core.commands.bt.behaviorTree
//import io.github.bionictigers.axiom.core.commands.bt.composites.Parallel
//import io.github.bionictigers.axiom.core.commands.bt.composites.ParallelPolicy
//import io.github.bionictigers.axiom.core.commands.bt.composites.Selector
//import io.github.bionictigers.axiom.core.commands.bt.composites.Sequence
//import io.github.bionictigers.axiom.core.commands.bt.decorators.Repeater
//import io.github.bionictigers.axiom.core.commands.bt.leaves.BtAction
//import io.github.bionictigers.axiom.core.commands.bt.leaves.Condition
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
//import org.firstinspires.ftc.teamcode.utils.ePower
//import kotlin.math.abs
//import kotlin.time.Duration.Companion.milliseconds
//
//@Autonomous
//class RedClose : LinearOpMode() {
//    private val startPose = Pose(609.6 * (5+1/5), 609.6 * 1, 45+90)
//    private val farShootPose = Pose(Distance.inch(20).mm, 609.6 * 2.5, 60+90)
//    private val shootClosePose = Pose(609.6 * 3.5, 609.6 * 2.5, 90 - 45+90)
//    private val queueClosePose = Pose(609.6 * 1.5, 609.6 * 2, 180)
//    private val grabClosePose = Pose(609.6 * 1.5, 609.6 * 0.5, 180)
//    private val queueMiddlePose = Pose(1211.37, 609.6 * 2, 180)
//    private val grabMiddlePose = Pose(390.60, 609.6 * 0.5, 180)
//    private val queueFarPose = Pose(989.85, 609.6 * 2, 180)
//    private val grabFarPose = Pose(384.92, 609.6 * 0.5, 180)
//
//
//    override fun runOpMode() {
//        val octoQuad = OctoQuad(hardwareMap, telemetry)
//        val odometry = Odometry(hardwareMap, telemetry, startPose)
//        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry, octoQuad)
//        val limelight = Limelight(hardwareMap, telemetry, false)
//        val transfer = Transfer(hardwareMap, octoQuad, telemetry)
//        val shooter = Shooter(hardwareMap, odometry, limelight, telemetry, false)
//        val intake = Intake(hardwareMap)
//
//        Scheduler.telemetry = telemetry
//        Scheduler.schedule(octoQuad, odometry, drivetrain, transfer, shooter)
//
//        val autoTree = behaviorTree("BlueClose Auto") {
//            sequence("Main Routine") {
//                add(
//                    shootRoutineFactory(
//                        name = "Preload Shoot",
//                        drivetrain = drivetrain,
//                        shooter = shooter,
//                        transfer = transfer,
//                        shootPose = shootPose,
//                        driveSpeed = 0.8,
//                        velocityTolerance = 80.0
//                    )
//                )
//                add(
//                    BtAction("Shutdown") {
//                        stopShooter(shooter)
//                        succeed()
//                    }
//                )
//            }
//        }.traced()
//
//        while (opModeInInit()) {
//            Scheduler.schedule()
//            Scheduler.tick()
//        }
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
//    private fun shootRoutineFactory(
//        name: String,
//        drivetrain: Drivetrain,
//        shooter: Shooter,
//        transfer: Transfer,
//        shootPose: Pose,
//        driveSpeed: Double = 1.0,
//        moveTimeoutMs: Long = 3200,
//        settleTimeMs: Long = 3000,
//        velocityTolerance: Double = 80.0,
//        velocityTimeoutMs: Long = 1500
//    ): BtCommand<*> {
//        return Sequence(
//            name,
//            listOf(
//                Parallel(
//                    "$name Shooter Move",
//                    children = listOf(
//                        moveToPoseFactory(
//                            name = "$name Drive",
//                            drivetrain = drivetrain,
//                            targetPose = shootPose,
//                            speed = driveSpeed,
//                            timeoutMs = moveTimeoutMs
//                        ),
//                        BtAction("$name Shooter Setup") {
//                            startShooter(shooter)
//                            succeed()
//                        }
//                    ),
//                    ParallelPolicy.REQUIRE_ONE
//                ),
//                Wait("$name Shooter Settle", settleTimeMs.milliseconds),
//                waitForShooterVelocityFactory(
//                    name = "$name Velocity Wait",
//                    shooter = shooter,
//                    tolerance = velocityTolerance,
//                    timeoutMs = velocityTimeoutMs
//                ),
//                Wait("$name Post-Velocity Wait", 1000.milliseconds),
//                BtAction("$name Shoot Prep") {
//                    Scheduler.schedule(transfer.shootPrep())
//                    succeed()
//                },
//                Wait("$name Shoot Prep Wait", 500.milliseconds),
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
//    private fun pickupRoutineFactory(
//        name: String,
//        drivetrain: Drivetrain,
//        transfer: Transfer,
//        pickupStartPose: Pose,
//        intake: Intake,
//        driveSpeed: Double = 1.0,
//        moveTimeoutMs: Long = 3200,
//        settleTimeMs: Long = 3000,
//        velocityTolerance: Double = 80.0,
//        velocityTimeoutMs: Long = 1500
//    ): BtCommand<*> {
//        return Sequence(
//            name,
//            listOf(
//                Parallel(
//                    "$name Move to start",
//                    children = listOf(
//                        moveToPoseFactory(
//                            name = "$name Drive",
//                            drivetrain = drivetrain,
//                            targetPose = pickupStartPose,
//                            speed = driveSpeed,
//                            timeoutMs = moveTimeoutMs
//                        ),
//                        BtAction( "$name Start intake" ) {
//                            Scheduler.schedule(intake.intake())
//                        }
//                    ),
//                ),
//                moveToPoseFactory(
//                    name = "$name Intake move",
//                    drivetrain = drivetrain,
//                    targetPose = pickupStartPose + Pose(0.0,TILE,0.0),
//                    speed = .4,
//                    timeoutMs = moveTimeoutMs
//                ),
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
//                // Skip checking mtp on the first tick because the scheduled
//                // move command has not entered yet.
//                return@BtAction
//            }
//
//            if (drivetrain.mtp == null) {
//                succeed()
//            }
//
//            if (meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds) {
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
//            val ready = targetVelocity >= 100000.0 ||
//                    targetVelocity <= 0.0 ||
//                    abs(shooter.currentVelocity - targetVelocity) <= tolerance
//            val timedOut = meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds
//            if (ready || timedOut) {
//                succeed()
//            }
//        }
//    }
//}