/**
 * Blocks for driving Kitronik :MOVE Motor (motors + ultrasonic only).
 */
//% weight=100 color=#00A654 icon="\uf1b9" block="MOVE Motor"
//% groups='["Ultrasonic", "Drive", "Setup", "Motor Control"]'
namespace Kitronik_Move_Motor {
    // PCA9632 registers for motor control on V1.0 / V2.0 boards.
    const CHIP_ADDR = 0x62
    const MODE_1_REG_ADDR = 0x00
    const MODE_2_REG_ADDR = 0x01
    const MOTOR_OUT_ADDR = 0x08

    const MODE_1_REG_VALUE = 0x00
    const MODE_2_REG_VALUE = 0x0D
    const MOTOR_OUT_VALUE = 0xAA

    // Ultrasonic conversion constants.
    const ULTRASONIC_V1_DIV_CM = 39
    const ULTRASONIC_V2_DIV_CM = 58
    const ULTRASONIC_V1_DIV_IN = 98
    const ULTRASONIC_V2_DIV_IN = 148

    export enum Motors {
        //% block="Left"
        MotorLeft = 0x04,
        //% block="Right"
        MotorRight = 0x02
    }

    export enum MotorDirection {
        //% block="Forward"
        Forward = 1,
        //% block="Reverse"
        Reverse = 2
    }

    export enum DriveDirections {
        //% block="Forward"
        Forward = 1,
        //% block="Reverse"
        Reverse = 2,
        //% block="Left"
        Left = 3,
        //% block="Right"
        Right = 4
    }

    export enum SpinDirections {
        //% block="Left"
        Left = 3,
        //% block="Right"
        Right = 4
    }

    export enum TurnRadii {
        //% block="Tight"
        Tight = 1,
        //% block="Standard"
        Standard = 2,
        //% block="Wide"
        Wide = 3
    }

    export enum Units {
        //% block="cm"
        Centimeters,
        //% block="inches"
        Inches
    }

    let initialised = false
    let moveMotorVersion = 0 // 10/20 use PCA9632, 31 uses WS2811 motor stream

    let rightMotorBias = 0
    let leftMotorBias = 0
    let turnTightness = 3

    let pwmReg1 = 0
    let pwmReg2 = 0
    let pwmReg3 = 0
    let pwmReg4 = 0

    // WS2811 motor stream (V3.1): GRB per motor, B channel used as brake light.
    let motorBuf = pins.createBuffer(6)
    const motorPin = DigitalPin.P12

    let cmEquationDivider = ULTRASONIC_V1_DIV_CM
    let inEquationDivider = ULTRASONIC_V1_DIV_IN
    const triggerPin = DigitalPin.P13
    const echoPin = DigitalPin.P14
    let units = Units.Centimeters

    function clampSpeed(speed: number): number {
        if (speed < 0) return 0
        if (speed > 100) return 100
        return speed
    }

    function clampPwm(value: number): number {
        if (value < 0) return 0
        if (value > 255) return 255
        return value
    }

    function setup(): void {
        let buf = pins.createBuffer(2)
        let readBuf = pins.createBuffer(1)

        // Board version detection used by original extension.
        basic.clearScreen()
        led.enable(false)
        pins.digitalWritePin(DigitalPin.P3, 1)
        basic.pause(100)
        if (pins.digitalReadPin(DigitalPin.P12) == 1) {
            pins.digitalWritePin(DigitalPin.P3, 0)
            basic.pause(100)
            if (pins.digitalReadPin(DigitalPin.P12) == 0) {
                moveMotorVersion = 20
            } else {
                moveMotorVersion = 0
            }
        }
        led.enable(true)

        if (moveMotorVersion == 0) {
            buf[0] = MODE_2_REG_ADDR
            buf[1] = MODE_2_REG_VALUE
            pins.i2cWriteBuffer(CHIP_ADDR, buf, false)
            readBuf = pins.i2cReadBuffer(CHIP_ADDR, 1, false)
            if (readBuf[0] != MODE_2_REG_VALUE) {
                moveMotorVersion = 31
            } else {
                moveMotorVersion = 10
            }
        }

        const sizeOfRam = control.ramSize()
        if (sizeOfRam >= 100000) {
            cmEquationDivider = ULTRASONIC_V2_DIV_CM
            inEquationDivider = ULTRASONIC_V2_DIV_IN
        } else {
            cmEquationDivider = ULTRASONIC_V1_DIV_CM
            inEquationDivider = ULTRASONIC_V1_DIV_IN
        }

        pins.digitalWritePin(triggerPin, 0)
        pins.digitalWritePin(echoPin, 0)

        // PCA motor init for V1.0 / V2.0 variants.
        if (moveMotorVersion != 31) {
            buf[0] = MODE_2_REG_ADDR
            buf[1] = MODE_2_REG_VALUE
            pins.i2cWriteBuffer(CHIP_ADDR, buf, false)

            buf[0] = MOTOR_OUT_ADDR
            buf[1] = MOTOR_OUT_VALUE
            pins.i2cWriteBuffer(CHIP_ADDR, buf, false)

            buf[0] = MODE_1_REG_ADDR
            buf[1] = MODE_1_REG_VALUE
            pins.i2cWriteBuffer(CHIP_ADDR, buf, false)
            basic.pause(1)
        }

        initialised = true
    }

    /**
     * Set ultrasonic distance units.
     * @param unit desired unit
     */
    //% subcategory="Sensors"
    //% group="Ultrasonic"
    //% blockId=kitronik_move_motor_ultrasonic_units
    //% block="measure distances in %unit"
    //% weight=100 blockGap=8
    export function setUltrasonicUnits(unit: Units): void {
        if (!initialised) setup()
        units = unit
    }

    /**
     * Measure distance using onboard ultrasonic sensor.
     * @param maxCmDistance maximum distance in cm
     */
    //% subcategory="Sensors"
    //% group="Ultrasonic"
    //% blockId=kitronik_move_motor_ultrasonic_measure
    //% block="measure distance"
    //% weight=95 blockGap=8
    export function measure(maxCmDistance = 500): number {
        if (!initialised) setup()

        pins.setPull(triggerPin, PinPullMode.PullNone)
        pins.digitalWritePin(triggerPin, 0)
        control.waitMicros(2)
        pins.digitalWritePin(triggerPin, 1)
        control.waitMicros(10)
        pins.digitalWritePin(triggerPin, 0)

        const pulse = pins.pulseIn(echoPin, PulseValue.High, maxCmDistance * 39)

        switch (units) {
            case Units.Centimeters:
                return Math.floor(pulse / cmEquationDivider)
            case Units.Inches:
                return Math.floor(pulse / inEquationDivider)
            default:
                return 0
        }
    }

    /**
     * Drive buggy in chosen direction.
     * @param direction movement direction
     * @param speed speed 0-100
     */
    //% subcategory=Motors
    //% group="Drive"
    //% blockId=kitronik_move_motor_drive
    //% weight=100 blockGap=8
    //% block="move %direction|at speed %speed"
    //% speed.min=0 speed.max=100
    export function move(direction: DriveDirections, speed: number): void {
        speed = clampSpeed(speed)

        switch (direction) {
            case DriveDirections.Forward:
                motorOn(Motors.MotorLeft, MotorDirection.Forward, speed)
                motorOn(Motors.MotorRight, MotorDirection.Forward, speed)
                break
            case DriveDirections.Reverse:
                motorOn(Motors.MotorLeft, MotorDirection.Reverse, speed)
                motorOn(Motors.MotorRight, MotorDirection.Reverse, speed)
                break
            case DriveDirections.Left:
                motorOn(Motors.MotorLeft, MotorDirection.Forward, speed / turnTightness)
                motorOn(Motors.MotorRight, MotorDirection.Forward, speed)
                break
            case DriveDirections.Right:
                motorOn(Motors.MotorLeft, MotorDirection.Forward, speed)
                motorOn(Motors.MotorRight, MotorDirection.Forward, speed / turnTightness)
                break
            default:
                stop()
        }
    }

    /**
     * Spin buggy on spot.
     * @param direction spin direction
     * @param speed speed 0-100
     */
    //% subcategory=Motors
    //% group="Drive"
    //% blockId=kitronik_move_motor_spin
    //% weight=95 blockGap=8
    //% block="spin %direction|at speed %speed"
    //% speed.min=0 speed.max=100
    export function spin(direction: SpinDirections, speed: number): void {
        speed = clampSpeed(speed)

        switch (direction) {
            case SpinDirections.Left:
                motorOn(Motors.MotorLeft, MotorDirection.Reverse, speed)
                motorOn(Motors.MotorRight, MotorDirection.Forward, speed)
                break
            case SpinDirections.Right:
                motorOn(Motors.MotorLeft, MotorDirection.Forward, speed)
                motorOn(Motors.MotorRight, MotorDirection.Reverse, speed)
                break
            default:
                stop()
        }
    }

    /** Stop both motors. */
    //% subcategory=Motors
    //% group="Drive"
    //% blockId=kitronik_move_motor_stop
    //% weight=90 blockGap=8
    //% block="stop"
    export function stop(): void {
        motorOff(Motors.MotorLeft)
        motorOff(Motors.MotorRight)
    }

    /**
     * Set motor bias to help straight line.
     * @param direction side to bias
     * @param balance value 0-10
     */
    //% subcategory=Motors
    //% group="Setup"
    //% blockId=kitronik_move_motor_motor_balance
    //% weight=85 blockGap=8
    //% block="bias to %direction by %balance"
    //% balance.min=0 balance.max=10
    export function motorBalance(direction: SpinDirections, balance: number): void {
        if (balance < 0) balance = 0
        else if (balance > 10) balance = 10

        leftMotorBias = 0
        rightMotorBias = 0

        switch (direction) {
            case SpinDirections.Left:
                leftMotorBias = Math.round(balance * 1.75)
                break
            case SpinDirections.Right:
                rightMotorBias = Math.round(balance * 1.75)
                break
        }
    }

    /**
     * Set turn radius preset for move left/right.
     * @param radius turn radius
     */
    //% subcategory=Motors
    //% group="Setup"
    //% blockId=kitronik_move_motor_motor_turn_radius
    //% weight=80 blockGap=8
    //% block="set turn radius %radius"
    export function turnRadius(radius: TurnRadii): void {
        switch (radius) {
            case TurnRadii.Tight:
                turnTightness = 8
                break
            case TurnRadii.Standard:
                turnTightness = 3
                break
            case TurnRadii.Wide:
                turnTightness = 1.6
                break
        }
    }

    /**
     * Advanced turn divider: inner wheel speed = speed / divider.
     * @param radiusDivider divider value
     */
    //% subcategory=Motors
    //% group="Setup"
    //% blockId=kitronik_move_motor_set_turn_radius
    //% weight=79 blockGap=8
    //% block="set advanced turn divider %radiusDivider"
    export function setTurnRadius(radiusDivider: number): void {
        if (radiusDivider <= 0) {
            turnTightness = 3
        } else {
            turnTightness = radiusDivider
        }
    }

    /**
     * Run one motor with direction and speed.
     * @param motor left or right motor
     * @param dir motor direction
     * @param speed speed 0-100
     */
    //% subcategory=Motors
    //% group="Motor Control"
    //% blockId=kitronik_move_motor_motor_on
    //% block="turn %motor|motor on direction %dir|speed %speed"
    //% weight=75 blockGap=8
    //% speed.min=0 speed.max=100
    export function motorOn(motor: Motors, dir: MotorDirection, speed: number): void {
        if (!initialised) setup()

        speed = clampSpeed(speed)
        const outputVal = clampPwm(Math.round(speed * 2.55))

        if (moveMotorVersion == 31) {
            let localBias = 0
            if (motor == Motors.MotorRight) localBias = rightMotorBias
            else localBias = leftMotorBias
            const motorVal = clampPwm(outputVal - localBias)

            switch (motor) {
                case Motors.MotorRight:
                    if (dir == MotorDirection.Forward) {
                        motorBuf[0] = motorVal
                        motorBuf[1] = 0
                    } else {
                        motorBuf[0] = 0
                        motorBuf[1] = motorVal
                    }
                    motorBuf[2] = outputVal == 0 ? 255 : 0
                    break
                case Motors.MotorLeft:
                    if (dir == MotorDirection.Forward) {
                        motorBuf[3] = 0
                        motorBuf[4] = motorVal
                    } else {
                        motorBuf[3] = motorVal
                        motorBuf[4] = 0
                    }
                    motorBuf[5] = outputVal == 0 ? 255 : 0
                    break
            }
            Kitronik_WS2811.sendBuffer(motorBuf, motorPin)
            return
        }

        const motorOut = pins.createBuffer(5)
        const motorJerk = pins.createBuffer(5)
        motorOut[0] = 0xA2
        motorJerk[0] = 0xA2

        switch (motor) {
            case Motors.MotorRight:
                if (dir == MotorDirection.Forward) {
                    pwmReg1 = 0
                    pwmReg2 = clampPwm(outputVal - rightMotorBias)
                    motorJerk[1] = 0
                    motorJerk[2] = 0xFF
                } else {
                    pwmReg1 = clampPwm(outputVal - rightMotorBias)
                    pwmReg2 = 0
                    motorJerk[1] = 0xFF
                    motorJerk[2] = 0
                }
                break
            case Motors.MotorLeft:
                if (dir == MotorDirection.Forward) {
                    pwmReg3 = clampPwm(outputVal - leftMotorBias)
                    pwmReg4 = 0
                    motorJerk[3] = 0xFF
                    motorJerk[4] = 0
                } else {
                    pwmReg3 = 0
                    pwmReg4 = clampPwm(outputVal - leftMotorBias)
                    motorJerk[3] = 0
                    motorJerk[4] = 0xFF
                }
                break
        }

        motorOut[1] = pwmReg1
        motorOut[2] = pwmReg2
        motorOut[3] = pwmReg3
        motorOut[4] = pwmReg4

        pins.i2cWriteBuffer(CHIP_ADDR, motorJerk, false)
        basic.pause(1)
        pins.i2cWriteBuffer(CHIP_ADDR, motorOut, false)
    }

    /**
     * Stop one motor.
     * @param motor selected motor
     */
    //% subcategory=Motors
    //% group="Motor Control"
    //% blockId=kitronik_move_motor_motor_off
    //% weight=70 blockGap=8
    //% block="turn off %motor| motor"
    export function motorOff(motor: Motors): void {
        if (!initialised) setup()

        if (moveMotorVersion == 31) {
            switch (motor) {
                case Motors.MotorRight:
                    motorBuf[0] = 0
                    motorBuf[1] = 0
                    motorBuf[2] = 255
                    break
                case Motors.MotorLeft:
                    motorBuf[3] = 0
                    motorBuf[4] = 0
                    motorBuf[5] = 255
                    break
            }
            Kitronik_WS2811.sendBuffer(motorBuf, motorPin)
            return
        }

        const motorOut = pins.createBuffer(5)
        motorOut[0] = 0xA2

        switch (motor) {
            case Motors.MotorRight:
                pwmReg1 = 0
                pwmReg2 = 0
                break
            case Motors.MotorLeft:
                pwmReg3 = 0
                pwmReg4 = 0
                break
        }

        motorOut[1] = pwmReg1
        motorOut[2] = pwmReg2
        motorOut[3] = pwmReg3
        motorOut[4] = pwmReg4
        pins.i2cWriteBuffer(CHIP_ADDR, motorOut, false)
    }
}
