package com.edinaftc.library.motortype;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev=383.6, gearing=13.7, maxRPM=435, orientation= Rotation.CCW)
@DeviceProperties(xmlTag="goBILDA5202SeriesMotor435", name="GoBILDA 5202 series435", builtIn = true)
@DistributorInfo(distributor="goBILDA_distributor", model="goBILDA-5202", url="https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motors/")
@ExpansionHubPIDFVelocityParams(P=18.0, I=0.5, D=5, F=12.1)
@ExpansionHubPIDFPositionParams(P=5.0)
public interface GoBILDA5202Series435 {
}
