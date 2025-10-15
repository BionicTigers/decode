package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.getByName

class Sorter(hardwareMap: HardwareMap, telemetry: Telemetry) : System() {
    override val name = "Vision"
     fun runOpMode(){}
    enum class Obelisk {
        PPG,
        PGP,
        GPP,
        NO_DETECTION
    }
    enum class BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    val limeLight = hardwareMap.getByName<Limelight3A>("limeLight")
    val motor = hardwareMap.getByName<DcMotorEx>("Ferris wheel")
    var colorSensor1 = hardwareMap.getByName<ColorSensor>("color sensor1")

    var contents = arrayOf(BallColor.NONE, BallColor.NONE, BallColor.NONE)
    val ticksPerRev = 8922
    var compartment1 = 0
    var compartment2 = 0
    var compartment3 = 0
    var zero = 0
    var one = ticksPerRev/3
    var two = ticksPerRev/3*2
    var slotAtEntrance = 0
    val output = Output(hardwareMap)
    fun sort(){
        if (colorSensor1. && slotAtEntrance == zero)
    }
    override val afterRun = Command.continuous {
        var colorSensorColor = Color(colorSensor1.red(),colorSensor1.blue(),colorSensor1.green())
        val purple = Color(0,0,0)
        val green = Color(0,0,0)
        if (colorSensorColor == purple) {
            contents[slotAtEntrance] = BallColor.PURPLE

        } else if (colorSensorColor == green){
            contents[slotAtEntrance] = BallColor.GREEN
        } else {
            contents[slotAtEntrance] = BallColor.NONE
        }

    }

    var obeliskCode = Obelisk.NO_DETECTION
        private set

    init {
        limeLight.setPollRateHz(100)
        limeLight.start()
    }

    fun seeApril() {
        limeLight.pipelineSwitch(0)
        val result = limeLight.latestResult
        val fiducials = result.fiducialResults
        fiducials?.forEach {
            obeliskCode = when (it?.fiducialId) {
                21 -> Obelisk.GPP
                22 -> Obelisk.PGP
                23 -> Obelisk.PPG
                else -> Obelisk.NO_DETECTION
            }


        }


    }

}


