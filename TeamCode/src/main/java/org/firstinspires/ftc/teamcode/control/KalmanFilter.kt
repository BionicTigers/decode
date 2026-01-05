package org.firstinspires.ftc.teamcode.control

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.mechanisms.LimeLight
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.motion.limeLight
import org.firstinspires.ftc.teamcode.utils.Angle

class KalmanFilter(val hardwareMap: HardwareMap, val telemetry: Telemetry, val getAngleVel: () -> Angle) {

    val currentVel: Double
        get() = getAngleVel().radians
  //   val limeLight = LimeLight(hardwareMap, telemetry)
  //   val currentAngle = Angle.radians(0)

    // var position = getAngleVel().radians + currentAngle.radians
   // val trust =
   // fun prediction() {
   //     position = getAngleVel().radians + currentAngle.radians
  //      telemetry.addData("Angle",position)
  //  }

  //  fun update(){
  //     if (limeLight.colorSeen){
  //         position = limeLight.getAngleFin() ?: 0.0
  //     }
  //      telemetry.addData("Angle", position)
  //  }

    var odometryNoise = 9.0 // Figure this out after tuning
    var visionNoise = 3.0 // Figure this out from testing

    fun prediction () {
        var unullify = limeLight.getAngleFin()
        if (unullify != null){
            val predictedAngle = unullify + currentVel
        }

    }



}

