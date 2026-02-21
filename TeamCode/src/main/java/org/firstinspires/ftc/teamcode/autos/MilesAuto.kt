package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.commands.bt.BtCommand
import io.github.bionictigers.axiom.core.commands.bt.behaviorTree
import io.github.bionictigers.axiom.core.commands.bt.leaves.BtAction
import io.github.bionictigers.axiom.core.commands.bt.traced
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.LimeLight
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Persistents
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.time.Duration.Companion.milliseconds

@Autonomous
class MilesAuto: LinearOpMode() {
    private val startPose = Pose(Distance.inch(9).mm, 609.6 * 2.2, 90)
    private val firstRow = Pose(609.6 * 1.5,609.6 * 3.5,180)
    private val secondRow = Pose(609.6 * 2.5,609.6 * 3.5,180)
    private val thirdRow = Pose(609.6 * 3.5,609.6 * 3.5,180)

    private val firstRowSide = Pose(609.6 * 1.5,609.6 * 6,180)
    private val secondRowSide = Pose(609.6 * 2.5,609.6 * 6,180)
    private val thirdRowSide = Pose(609.6 * 3.5,609.6 * 5.6,180)

    override fun runOpMode() {
        val octoQuad = OctoQuad(hardwareMap, telemetry)
        val odometry = Odometry(hardwareMap, telemetry, startPose)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry, octoQuad)
        val intake = Intake(hardwareMap, octoQuad)
        val kicker = Kicker(hardwareMap, telemetry)
        val sorter = Sorter(hardwareMap, kicker, telemetry, octoQuad)
        val limeLight = LimeLight(hardwareMap, telemetry )
        val output = Output(hardwareMap, kicker, sorter, telemetry, odometry, octoQuad, true, limeLight )


        kicker.servo.position = .35

        Scheduler.telemetry = telemetry
        Scheduler.schedule(octoQuad, odometry, drivetrain, intake, kicker, sorter, output)

        val autoTree = behaviorTree("miles' auto"){
            sequence("maon routine"){
                add(BtAction {
                    sorter.isOutput = true
                    Scheduler.schedule(sorter.moveForward())
                    succeed()
                }
                );
                add(BtAction {
                    Scheduler.schedule(output.shoot())
                }
                );
                add(BtAction {
                    Scheduler.schedule(sorter.moveForward())
                }
                );
                add(BtAction {
                    pickBallsUpRowOne( "miles",
                        drivetrain,
                        intake,
                        sorter)
                }

                );
                add(BtAction {
                    moveToShootingPosition(
                        "move to position",
                        drivetrain,
                        output,
                        sorter
                        )
                }
                )

            }
        }.traced()
        waitForStart()
        if (isStopRequested) {
            Scheduler.clear()
            Persistents.currentPose = odometry.position
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
        Persistents.currentPose = odometry.position
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

    fun switchFerrisWheel(
        name: String,
        sorter: Sorter
    ){
        BtAction("miles"){
            Scheduler.schedule(sorter.moveForward())
        }
    }
    val sorterForward = switchFerrisWheel("miles", Sorter(
        hardwareMap,
        Kicker(
            hardwareMap,
            telemetry
        ),
        telemetry,
        OctoQuad(
            hardwareMap,
            telemetry
        ),
    )

    )
    fun pickBallsUpRowOne(
        name: String,
        drivetrain: Drivetrain,
        intake: Intake,
        sorter: Sorter
    ){
        Scheduler.schedule(intake.intake())
        moveToPoseFactory(
            "Drive to pickup",
            drivetrain,
            firstRow,
            0.5,
            2600
        )
        moveToPoseFactory(
            "Pickup balls",
            drivetrain,
            firstRowSide,
            0.5,
            2600
        )

        if (sorter.isGreenSeen == false){
            BtAction{
                fail()
                pickBallsUpRowTwo( "miles",
                    drivetrain,
                    intake,
                    sorter)
            }
        }
    }
    fun pickBallsUpRowTwo(
        name: String,
        drivetrain: Drivetrain,
        intake: Intake,
        sorter: Sorter
    ){
        Scheduler.schedule(intake.intake())
        moveToPoseFactory(
            "Drive to pickup",
            drivetrain,
            secondRow,
            0.5,
            2600
        )
        moveToPoseFactory(
            "Pickup balls",
            drivetrain,
            secondRowSide,
            0.5,
            2600
        )




        }
    }
    fun pickBallsUpRowThree(
        name: String,
        drivetrain: Drivetrain,
        intake: Intake,
        sorter: Sorter
    ){
        Scheduler.schedule(intake.intake())
        moveToPoseFactory(
            "Drive to pickup",
            drivetrain,
            thirdRow,
            0.5,
            2600
        )
        moveToPoseFactory(
            "Pickup balls",
            drivetrain,
            thirdRowSide,
            0.5,
            2600
        )

        if (!sorter.isGreenSeen){
            BtAction{
                fail()
            }
        }
    }
    fun moveToShootingPosition(
        name: String,
        drivetrain: Drivetrain,
        output: Output,
        sorter: Sorter
    ){
        moveToPoseFactory(
            "go to shooting position",
            drivetrain,
            startPose,
            0.5,
            2600
            )
        Scheduler.schedule(output.shoot())
        Scheduler.schedule(sorter.moveForward())
        Scheduler.schedule(output.shoot())
        Scheduler.schedule(sorter.moveForward())
        Scheduler.schedule(output.shoot())
        Scheduler.schedule(sorter.moveForward())
    }
    }
