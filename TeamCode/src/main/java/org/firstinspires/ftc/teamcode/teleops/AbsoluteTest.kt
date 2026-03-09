package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3
import org.firstinspires.ftc.teamcode.utils.getByName

@TeleOp
class AbsoluteTest : LinearOpMode() {
    override fun runOpMode() {
        val octoQuad = hardwareMap.getByName<OctoQuadFWv3>("octoquad")
        octoQuad.channelBankConfig = OctoQuadFWv3.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH
        octoQuad.setSingleChannelPulseWidthParams(7, 1, 1024)

        waitForStart()

        while(opModeIsActive()) {
            val a = octoQuad.readAllEncoderData()
            telemetry.addData("Reading", a.positions[7])
            telemetry.addData("Crc", a.crcOk)
            telemetry.update()
        }
    }
}