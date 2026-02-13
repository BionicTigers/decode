//package org.firstinspires.ftc.teamcode.mechanisms
//
//import com.qualcomm.hardware.limelightvision.Limelight3A
//import com.qualcomm.robotcore.hardware.HardwareMap
//import io.github.bionictigers.axiom.core.commands.Command
//
//import io.github.bionictigers.axiom.core.commands.System
//import io.github.bionictigers.axiom.core.scheduler.Scheduler.telemetry
//import org.firstinspires.ftc.robotcore.external.Telemetry
//import org.firstinspires.ftc.teamcode.utils.Distance
//import org.firstinspires.ftc.teamcode.utils.getByName
//import kotlin.math.atan2
//
//
//class LimeLight(hardwareMap: HardwareMap, telemetry: Telemetry): System() {
//
//
//    override val name = "Vision"
//    enum class Obelisk {
//        PPG,
//        PGP,
//        GPP,
//
//        RED,
//
//        BLUE,
//        NO_DETECTION
//    }
//    val limeLight = hardwareMap.getByName<Limelight3A>("limeLight")
//    var obeliskCode = Obelisk.NO_DETECTION
//        private set
//
//    init {
//        limeLight.setPollRateHz(100)
//        limeLight.start()
//    }
//
//
//
//    fun getAngleFin(): Double? {
//        limeLight.pipelineSwitch(0)
//        val result = limeLight.latestResult
//        val fiducials = result.fiducialResults
//        fiducials?.forEach {
//            obeliskCode = when (it?.fiducialId) {
//
//                21 -> Obelisk.GPP
//                22 -> Obelisk.PGP
//                23 -> Obelisk.PPG
//
//
//                else -> Obelisk.NO_DETECTION
//            }
//
//
//        }
//
//
//
//        telemetry?.addData("ID", fiducials.map { it.fiducialId }.toString())
//
//
//        var tx = result.getTx()
//        var ty = result.getTy()
//        if(ty == 0.0){
//            telemetry?.addData("You did it", ty * 0.03937)
//        }
//        if (ty != 0.0){
//            telemetry?.addData("Keep going", ty * 0.03937)
//        }
//
//
//
//
//
//        return tx
//    }
//
//    override val apply = SystemCommand.continuous {
//        getAngleFin()
//    }
//}