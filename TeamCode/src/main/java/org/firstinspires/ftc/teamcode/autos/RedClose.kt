package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.commands.bt.BtCommand
import io.github.bionictigers.axiom.core.commands.bt.behaviorTree
import io.github.bionictigers.axiom.core.commands.bt.composites.Parallel
import io.github.bionictigers.axiom.core.commands.bt.composites.ParallelPolicy
import io.github.bionictigers.axiom.core.commands.bt.composites.Selector
import io.github.bionictigers.axiom.core.commands.bt.composites.Sequence
import io.github.bionictigers.axiom.core.commands.bt.decorators.Repeater
import io.github.bionictigers.axiom.core.commands.bt.leaves.BtAction
import io.github.bionictigers.axiom.core.commands.bt.leaves.Condition
import io.github.bionictigers.axiom.core.commands.bt.leaves.Wait
import io.github.bionictigers.axiom.core.commands.bt.traced
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.math.abs
import kotlin.time.Duration.Companion.milliseconds

@Autonomous
class RedClose : LinearOpMode() {
    private val startPose = Pose(609.6 * (5+1/5), 609.6 * 1, 45+90)
    private val farShootPose = Pose(Distance.inch(20).mm, 609.6 * 2.5, 60+90)
    private val shootClosePose = Pose(609.6 * 3.5, 609.6 * 2.5, 90 - 45+90)
    private val queueClosePose = Pose(609.6 * 1.5, 609.6 * 2, 180)
    private val grabClosePose = Pose(609.6 * 1.5, 609.6 * 0.5, 180)
    private val queueMiddlePose = Pose(1211.37, 609.6 * 2, 180)
    private val grabMiddlePose = Pose(390.60, 609.6 * 0.5, 180)
    private val queueFarPose = Pose(989.85, 609.6 * 2, 180)
    private val grabFarPose = Pose(384.92, 609.6 * 0.5, 180)

    override fun runOpMode() {
        val octoQuad = OctoQuad(hardwareMap, telemetry)
        val odometry = Odometry(hardwareMap, telemetry, startPose)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry, octoQuad)
        val intake = Intake(hardwareMap, octoQuad)
        val kicker = Kicker(hardwareMap, telemetry)
        val sorter = Sorter(hardwareMap, kicker, telemetry, octoQuad)
        val output = Output(hardwareMap, kicker, sorter, telemetry, odometry, octoQuad, false)

        kicker.servo.position = .35

        Scheduler.telemetry = telemetry
        Scheduler.schedule(octoQuad, odometry, drivetrain, intake, kicker, sorter, output)

        val autoTree = behaviorTree("BlueFar Auto") {
            sequence("Main Routine") {
                add(
                    BtAction {
                        sorter.isOutput = true
                        Scheduler.schedule(sorter.moveForward())
                        succeed()
                    }
                )
                add(
                    shootRoutineFactory(
                        name = "Preload Shoot",
                        drivetrain = drivetrain,
                        output = output,
                        sorter = sorter,
                        kicker = kicker,
                        shootPose = shootClosePose,
                        useCloseVelocity = false
                    )
                )
                add(
                    cycleFactory(
                        name = "Close Cycle",
                        drivetrain = drivetrain,
                        intake = intake,
                        output = output,
                        sorter = sorter,
                        kicker = kicker,
                        queuePose = queueClosePose,
                        grabPose = grabClosePose,
                        shootPose = shootClosePose,
                        useCloseVelocity = true
                    )
                )
                add(
                    cycleFactory(
                        name = "Middle Cycle",
                        drivetrain = drivetrain,
                        intake = intake,
                        output = output,
                        sorter = sorter,
                        kicker = kicker,
                        queuePose = queueMiddlePose,
                        grabPose = grabMiddlePose,
                        shootPose = shootClosePose,
                        useCloseVelocity = true
                    )
                )
                add(
                    cycleFactory(
                        name = "Far Cycle",
                        drivetrain = drivetrain,
                        intake = intake,
                        output = output,
                        sorter = sorter,
                        kicker = kicker,
                        queuePose = queueFarPose,
                        grabPose = grabFarPose,
                        shootPose = shootClosePose,
                        useCloseVelocity = true
                    )
                )
                add(
                    moveToPoseFactory(
                        "Park",
                        drivetrain,
                        queueClosePose,
                        speed = 1.0,
                        timeoutMs = 3000
                    )
                )
                add(
                    BtAction("Shutdown") {
                        Scheduler.schedule(output.stop())
                        Scheduler.schedule(intake.stop())
                        succeed()
                    }
                )
            }
        }.traced()

        waitForStart()
        if (isStopRequested) {
            Scheduler.clear()
            return
        }

        Scheduler.schedule(autoTree)

        while (opModeIsActive()) {
            Scheduler.tick()
            telemetry.addData("Auto Node Status", autoTree.treeStatus)
            telemetry.addData("Transfer Occupied", sorter.occupiedBays)
            telemetry.update()
        }

        Scheduler.clear()
    }

    private fun cycleFactory(
        name: String,
        drivetrain: Drivetrain,
        intake: Intake,
        output: Output,
        sorter: Sorter,
        kicker: Kicker,
        queuePose: Pose,
        grabPose: Pose,
        shootPose: Pose,
        useCloseVelocity: Boolean
    ): BtCommand<*> {
        return io.github.bionictigers.axiom.core.commands.bt.composites.Sequence(
            name,
            listOf(
                pickupRoutineFactory(
                    name = "$name Pickup",
                    drivetrain = drivetrain,
                    intake = intake,
                    output = output,
                    sorter = sorter,
                    queuePose = queuePose,
                    grabPose = grabPose
                ),
                shootRoutineFactory(
                    name = "$name Shoot",
                    drivetrain = drivetrain,
                    output = output,
                    sorter = sorter,
                    kicker = kicker,
                    shootPose = shootPose,
                    useCloseVelocity = useCloseVelocity
                )
            )
        )
    }

    private fun pickupRoutineFactory(
        name: String,
        drivetrain: Drivetrain,
        intake: Intake,
        output: Output,
        sorter: Sorter,
        queuePose: Pose,
        grabPose: Pose
    ): BtCommand<*> {
        return io.github.bionictigers.axiom.core.commands.bt.composites.Sequence(
            name,
            listOf(
                moveToPoseFactory(
                    "$name Queue Move",
                    drivetrain,
                    queuePose,
                    speed = 0.5,
                    timeoutMs = 2600
                ),
                BtAction("$name Intake Setup") {
                    sorter.isOutput = false
//                    sorter.move()
                    Scheduler.schedule(output.stop())
                    Scheduler.schedule(intake.intake())
                    succeed()
                },
                moveToPoseFactory(
                    "$name Grab Move",
                    drivetrain,
                    grabPose,
                    speed = 0.35,
                    timeoutMs = 3200
                ),
                intakeFillRoutineFactory(
                    name = "$name Fill Transfer",
                    sorter = sorter,
                    targetOccupiedBays = 3,
                    timeoutMs = 2200
                ),
                BtAction("$name Intake Stop") {
                    Scheduler.schedule(intake.stop())
                    succeed()
                }
            )
        )
    }

    private fun shootRoutineFactory(
        name: String,
        drivetrain: Drivetrain,
        output: Output,
        sorter: Sorter,
        kicker: Kicker,
        shootPose: Pose,
        useCloseVelocity: Boolean,
        shots: Int = 3
    ): BtCommand<*> {
        val children = mutableListOf<BtCommand<*>>(
            Parallel(
                "$name Shooter Move", children = listOf(
                    moveToPoseFactory(
                        "$name Drive",
                        drivetrain,
                        shootPose,
                        speed = 0.8,
                        timeoutMs = 3200
                    ),
                    BtAction("$name Shooter Setup") {
                        sorter.isOutput = true
                        sorter.move()
                        Scheduler.schedule(output.shoot())
                        succeed()
                    },
                ), ParallelPolicy.REQUIRE_ONE
            ),
            Wait("$name Shooter Settle", 3000.milliseconds),
            waitForShooterVelocityFactory(
                "$name Velocity Wait",
                output,
                tolerance = 60.0,
                timeoutMs = 1500
            )
        )

        if (shots > 1) {
            children += Repeater(
                "$name Repeat Shot",
                singleShotFactory("$name Repeated Shot", kicker, sorter, rotateAfter = true),
                times = shots - 1,
                stopOnFailure = false
            )
        }

        children += singleShotFactory("$name Final Shot", kicker, sorter, rotateAfter = false)
        children += BtAction("$name Shooter Stop") {
            Scheduler.schedule(output.stop())
            succeed()
        }

        return io.github.bionictigers.axiom.core.commands.bt.composites.Sequence(name, children)
    }

    private fun singleShotFactory(
        name: String,
        kicker: Kicker,
        sorter: Sorter,
        rotateAfter: Boolean
    ): BtCommand<*> {
        return io.github.bionictigers.axiom.core.commands.bt.composites.Sequence(
            name,
            listOf(
                Selector(
                    "$name Kick Check",
                    listOf(
                        Sequence(
                            "$name Kick If Loaded",
                            listOf(
                                Condition("$name Has Ball") {
                                    // Allow blind firing when tracking is unknown (all bays empty).
                                    sorter.hasBallAtPosition(Sorter.SlotPosition.Output) || sorter.occupiedBays == 0
                                },
                                BtAction("$name Kick") {
                                    Scheduler.schedule(kicker.kick())
                                    succeed()
                                }
                            )
                        ),
                        BtAction("$name Skip Kick") { succeed() }
                    )
                ),
                Wait("$name Kick Hold", 400.milliseconds),
                BtAction("$name Rotate Transfer") {
                    if (rotateAfter) {
                        Scheduler.schedule(sorter.moveForward())
                    }
                    succeed()
                },
                Wait("$name Transfer Hold", 1000.milliseconds)
            )
        )
    }

    private fun moveToPoseFactory(
        name: String,
        drivetrain: Drivetrain,
        targetPose: Pose,
        speed: Double,
        timeoutMs: Long
    ): BtCommand<*> {
        var started = false
        return BtAction(name) {
            if (!started) {
                started = true
                Scheduler.schedule(drivetrain.moveToPosition(targetPose, speed))
                // Skip checking mtp on the first tick — the moveToPosition
                // command's enter{} hasn't run yet so mtp is still null.
                return@BtAction
            }

            if (drivetrain.mtp == null) {
                println("move success")
                succeed()
            }

            if (meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds) {
                println("move failed")
//                drivetrain.mtp = null
                succeed()
            }
        }
    }

    private fun waitForShooterVelocityFactory(
        name: String,
        output: Output,
        tolerance: Double,
        timeoutMs: Long
    ): BtCommand<*> {
        return BtAction(name) {
            val ready = abs(output.currentVel - output.targetVelocity) <= tolerance
            val timedOut = meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds
            if (ready || timedOut) {
                succeed()
            }
        }
    }

    private fun intakeFillRoutineFactory(
        name: String,
        sorter: Sorter,
        targetOccupiedBays: Int,
        timeoutMs: Long
    ): BtCommand<*> {
        var hadBallInIntakeLastTick = false
        return BtAction(name) {
            val hasBallInIntake = sorter.hasBallAtPosition(Sorter.SlotPosition.Intake)
            if (hasBallInIntake && !hadBallInIntakeLastTick) {
                Scheduler.schedule(sorter.moveForward())
            }
            hadBallInIntakeLastTick = hasBallInIntake

            if (sorter.occupiedBays >= targetOccupiedBays || meta.enteredAt.elapsedNow() >= timeoutMs.milliseconds) {
                succeed()
            }
        }
    }
}