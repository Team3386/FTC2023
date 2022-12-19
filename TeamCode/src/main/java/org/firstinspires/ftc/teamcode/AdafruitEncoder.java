package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@SuppressWarnings("WeakerAccess")
@I2cDeviceType
@DeviceProperties(name = "Adafruit Encoder", description = "Lol", xmlTag = "AdafruitXDEncoder", builtIn = false)

public class AdafruitEncoder extends I2cDeviceSynchDevice<I2cDeviceSynch> implements I2cAddrConfig {


    @Override
    public Manufacturer getManufacturer() {

        return Manufacturer.Adafruit;
    }

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0x36);

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {

        return "Adafruit Encoder";
    }


    public AdafruitEncoder(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }


    protected short readShort(int reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg, 2));
    }

    protected double readDouble(int reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg, 4));
    }

    static final byte SEESAW_ENCODER_BASE = 0x11,
            SEESAW_ENCODER_STATUS = 0x00,
            SEESAW_ENCODER_INTENSET = 0x10,
            SEESAW_ENCODER_INTENCLR = 0x20,
            SEESAW_ENCODER_POSITION = 0x30,
            SEESAW_ENCODER_DELTA = 0x40;
    static final int
            SEESAW_HW_ID_CODE_SAMD09 = 0x55,
            SEESAW_HW_ID_CODE_TINY8X7 = 0x87;

    protected void setOptimalReadWindow() {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                SEESAW_ENCODER_BASE,
                0x30,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    I2cDeviceSynchSimple _device;
    boolean _flow;
    boolean initialized = false;


    public double getEncoderPosition() {
        return readDouble(SEESAW_ENCODER_POSITION);
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        this.deviceClient.setI2cAddress(newAddress);
    }

    @Override
    public I2cAddr getI2cAddress() {
        return this.deviceClient.getI2cAddress();
    }
}
