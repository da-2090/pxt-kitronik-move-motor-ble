/**
 * Blocks for driving the Kitronik :MOVE Motor Buggy
 */
//% weight=100 color=#00A654 icon="\uf1b9" block="MOVE Motor"
//% groups='["Ultrasonic","Line Following","Drive", "Setup", "Motor Control"]'
namespace Kitronik_Move_Motor {
    //Constants for PCA9632 Driver IC
    let CHIP_ADDR = 0x62 // CHIP_ADDR is the standard chip address for the PCA9632, datasheet refers to LED control but chip is used for PWM to motor driver
    let MODE_1_REG_ADDR = 0x00 //mode 1 register address
    let MODE_2_REG_ADDR = 0x01  //mode 2 register address
    let MOTOR_OUT_ADDR = 0x08  //MOTOR output register address

    let MODE_1_REG_VALUE = 0x00 //setup to normal mode and not to respond to sub address
    let MODE_2_REG_VALUE = 0x0D  //Setup to make changes on ACK, outputs set to Totem poled to drive the motor driver chip
    let MOTOR_OUT_VALUE = 0xAA  //Outputs set to be controled PWM registers

    //Define of the two different values required in conversion equation for the ultrasonic to measure accurately
    let ULTRASONIC_V1_DIV_CM = 39
    let ULTRASONIC_V2_DIV_CM = 58
    let ULTRASONIC_V1_DIV_IN = 98
    let ULTRASONIC_V2_DIV_IN = 148
    
    /*GENERAL*/
    export enum OnOffSelection {
        //% block="on"
        On = 1,
        //% block="off"
        Off = 0
    }

    /* ZIPLEDS*/
    //Well known colors for ZIP LEDs
    export enum ZipLedColors {
        //% block=red
        Red = 0xFF0000,
        //% block=orange
        Orange = 0xFFA500,
        //% block=yellow
        Yellow = 0xFFFF00,
        //% block=green
        Green = 0x00FF00,
        //% block=blue
        Blue = 0x0000FF,
        //% block=indigo
        Indigo = 0x4b0082,
        //% block=violet
        Violet = 0x8a2be2,
        //% block=purple
        Purple = 0xFF00FF,
        //% block=white
        White = 0xFFFFFF,
        //% block=black
        Black = 0x000000
    }

    /*MOTORS*/
    // List of motors for the motor blocks to use. These correspond to the register offsets in the PCA9832 driver IC.
    export enum Motors {
        //% block="Left"
        MotorLeft = 0x04,
        //% block="Right"
        MotorRight = 0x02
    }
    // Directions the motors can rotate.
    export enum MotorDirection {
        //% block="Forward"
        Forward = 1,
        //% block="Reverse"
        Reverse = 2
    }
    // directions the :MOVE motor can drive. Implicit moving forward in the turns 
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
    //directions the :MOVE motor can spin on the spot. 
    export enum SpinDirections {
        //%block="Left"
        Left = 3,
        //%block="Right"
        Right = 4
    }
    
    export enum TurnRadii {
        //%block="Tight"
        Tight = 1,
        //%block="Standard"
        Standard = 2,
        //%block="Wide"
        Wide = 3
    }
    
    /*ULTRASONIC*/
    // Units for ultrasonic sensors to measure
    export enum Units {
        //% block="cm"
        Centimeters,
        //% block="inches"
        Inches
    }

    /*LINE FOLLOWING*/
    // Selection of line following sensor
    export enum LfSensor {
        //% block="Left"
        Left,
        //% block="Right"
        Right
    }
    //Light level detection mode selection
    export enum LightSelection {
        //% block="Light"
        Light,
        //% block="Dark"
        Dark
    }
    
    //Detection mode selection
    export enum DetectorSensitivity 
    {
        //% block="Low"
        Low,
        //% block="Medium"
        Medium,
        //% block="High"
        High
    }

    //Pin selection from expansion pins
    export enum PinSelection
    {
        //% block="P15"
        P15,
        //% block="P16"
        P16
    }

    //Pin selection from expansion pins
    export enum ServoSelection
    {
        //% block="servo 1"
        servo1,
        //% block="servo 2"
        servo2
    }

    let initalised = false //a flag to allow us to initialise without the user having to explicitly call the initialisation routine
    let moveMotorVersion = 0
    //Motor global variables to allow user to 'bias' the motors to drive the :MOVE motor in a straight line
    let rightMotorBias = 0
    let leftMotorBias = 0
    let turnTightness = 4
    let latestMotorBuf = pins.createBuffer(6) // Store the most recent data sent to the WS2811 driver so that we can see the latest speed settings
    //Sound global variables
    let sirenOn = false
    //Ultrasonic global variables
    let cmEquationDivider = ULTRASONIC_V1_DIV_CM
    let inEquationDivider = ULTRASONIC_V1_DIV_IN
    let triggerPin = DigitalPin.P13
    let echoPin = DigitalPin.P14
    let units = Units.Centimeters
    //Line following sensors global variables
    let rightLfOffset = 0
    let leftLfOffset = 0
    
    //let uBitVersion = 0

   // function hardwareVersion(): number {
    //    return uBitVersion;
    //}
    
    /*
	This function checks the version of :MOVE Motor PCB and carries out different setup routines depending on the version number.
	It is called from other blocks, so never needs calling directly.
    */
    function setup(): void {
        let buf = pins.createBuffer(2)
        let readBuf = pins.createBuffer(1)
        // Pin 3 toggled while Pin 12 reads the toggle - this is to determine the version of the board for which line following sensor is attached (V1.0 or V2.0)
        basic.clearScreen()
        //led.enable(false) // Disable uBit LED display as Pin 3 controls part it
        pins.digitalWritePin(DigitalPin.P3, 1)
        basic.pause(100)
        if (pins.digitalReadPin(DigitalPin.P12) == 1) {
            pins.digitalWritePin(DigitalPin.P3, 0)
            basic.pause(100)
            if (pins.digitalReadPin(DigitalPin.P12) == 0) {
                moveMotorVersion = 20
            } else {
                moveMotorVersion = 0 // Set to 0 as still need to identify between V1.0 and V3.1
            }
        }
        //led.enable(true) // Enable the uBit LED display again

        // If the toggle check does not identify the board as V1.3, there needs to be a check between V1.0 and V3.1
        // Attempting to write/read a register on the PCA9632 IC will identify whether the IC is there or not (no PCA9632 on V3.1)
        // Note: This check relies on the micro:bit not throwing an error when an I2C address is used which isn't present on the I2C bus
        if (moveMotorVersion == 0) {
            buf[0] = MODE_2_REG_ADDR
            buf[1] = MODE_2_REG_VALUE
            pins.i2cWriteBuffer(CHIP_ADDR, buf, false)
            readBuf = pins.i2cReadBuffer(CHIP_ADDR, 1, false)
            let readValue = readBuf[0]

            if (readValue != MODE_2_REG_VALUE) {
                moveMotorVersion = 31
            }
            else {
                moveMotorVersion = 10
            }
        }
        
        //If version number is 1.0 (10) run the equalise sensor code
        if (moveMotorVersion == 10){
            equaliseSensorOffsets()
        }
        
        //determine which version of microbit is being used. From this the correct value is used in the equation to convert pulse to distance
        //uBitVersion = hardwareVersion()
        let sizeOfRam = control.ramSize()
        if (sizeOfRam >= 100000)
        {
            cmEquationDivider = ULTRASONIC_V2_DIV_CM
            inEquationDivider = ULTRASONIC_V2_DIV_IN
        }
        else
        {
            cmEquationDivider = ULTRASONIC_V1_DIV_CM
            inEquationDivider = ULTRASONIC_V1_DIV_IN
        }
        pins.digitalWritePin(DigitalPin.P13, 0) //set the ultrasonic pin low ready to trigger
        pins.digitalWritePin(DigitalPin.P14, 0) //set the ultrasonic pin low ready for return pulse

        // Setup the PCA9632 IC if NOT Version 3.1
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

        initalised = true // We have setup, so don't come in here again.
    }

    //////////////
    //  SENSORS //
    //////////////

    /**
     * Set the distance measurement units to cm or inches (cm is default)
     * @param unit desired conversion unit
     */
    //% subcategory="Sensors"
    //% group="Ultrasonic"
    //% blockId=kitronik_move_motor_ultrasonic_units
    //% block="measure distances in %unit"
    //% weight=100 blockGap=8
    export function setUltrasonicUnits(unit: Units): void {
        if (initalised == false) {
            setup()
        }
        units = unit
    }
    
    /**
     * Measure the echo time (after trigger) and converts to cm or inches and returns as an int
     * @param maxCmDistance maximum distance in centimeters (default is 500)
     */
    //% subcategory="Sensors"
    //% group="Ultrasonic"
    //% blockId=kitronik_move_motor_ultrasonic_measure
    //% block="measure distance"
    //% weight=95 blockGap=8
    export function measure(maxCmDistance = 500): number {
        if (initalised == false) {
            setup()
        }
        // send pulse
        pins.setPull(triggerPin, PinPullMode.PullNone);
        pins.digitalWritePin(triggerPin, 0);
        control.waitMicros(2);
        pins.digitalWritePin(triggerPin, 1);
        control.waitMicros(10);
        pins.digitalWritePin(triggerPin, 0);

        // read pulse
        const pulse = pins.pulseIn(echoPin, PulseValue.High, maxCmDistance * 39);
        //From the HC-SR04 datasheet the formula for calculating distance is "microSecs of pulse"/58 for cm or "microSecs of pulse"/148 for inches.
        //When measured actual distance compared to calculated distanceis not the same.  There must be an timing measurement with the pulse.
        //values have been changed to match the correct measured distances so 58 changed to 39 and 148 changed to 98
        //added variable that is set depending on the version of hardware used in the Move Motor 
        switch (units) {
            case Units.Centimeters: return Math.idiv(pulse, cmEquationDivider);
            case Units.Inches: return Math.idiv(pulse, inEquationDivider);
            //case Units.Centimeters: return Math.idiv(pulse, 39);
            //case Units.Inches: return Math.idiv(pulse, 98);
            default: return 0;
        }
    }
    
    
    /**
    This function allows us to read the difference in the sensor outputs. 
    It assumes that both are over a similar surface, and hence any offset is a fixed offset caused by component tolerances.
    If the sensors are over different surfaces it will reuslt in a false offset reading,and pants perfomance.
    We do this to help hide the complexity of tolerance and similar from novice users, 
    but if you are an expert reading this comment then feel free to play with this functionand see what it does.
    **/
    export function equaliseSensorOffsets(): void {
        let rightLineSensor = pins.analogReadPin(AnalogPin.P1)
        let leftLineSensor = pins.analogReadPin(AnalogPin.P2)
        let Offset = ((Math.abs(rightLineSensor-leftLineSensor))/2)
        if (leftLineSensor > rightLineSensor) {
            leftLfOffset = -Offset
            rightLfOffset = Offset
        } else {
        leftLfOffset = Offset
            rightLfOffset = -Offset
        }
        /*if (leftLineSensor > rightLineSensor) {
            rightLfOffset = leftLineSensor - rightLineSensor
        } else if (leftLineSensor < rightLineSensor) {
            leftLfOffset = rightLineSensor - leftLineSensor
        }*/
    }

    /**
    * Read sensor block allows user to read the value of the sensor (returns value in range 0-1023)
    * @param pinSelected is the selection of pin to read a particular sensor
    */
    //% subcategory="Sensors"
    //% group="Line Following"
    //% blockId=kitronik_move_motor_line_follower_read_sensor
    //% block="%pinSelected| line following sensor value"
    //% weight=85 blockGap=8
    export function readSensor(sensorSelected: LfSensor) {
        if (initalised == false) {
            setup()
        }
        let value = 0
        if (sensorSelected == LfSensor.Left) {
            value = pins.analogReadPin(AnalogPin.P2) + leftLfOffset
        }
        else if (sensorSelected == LfSensor.Right) {
            value = pins.analogReadPin(AnalogPin.P1) + rightLfOffset
        }
        return value;
    }


    //////////////
    //  MOTORS  //
    //////////////
    // These variables hold the register values for the 4 PWM registers.
    // Each pair of these controls a motor. We do it this way so we can do an auto increment write to the chip.
    let PWMReg1 = 0
    let PWMReg2 = 0
    let PWMReg3 = 0
    let PWMReg4 = 0

    // Note: WS2811 takes the data in GRB format
    let motorBuf = pins.createBuffer(6) // WS2811 ICs, each with RGB (RG = Motor, B = Brake)
    let motorPin = DigitalPin.P12

    /**
     * Drives the :MOVE motor in the specified direction. Turns have a small amount of forward motion.
     * @param direction Direction to move in
     * @param speed How fast to go (0-100)
     */
    //% subcategory=Motors
    //% group="Drive"
    //% blockId=kitronik_move_motor_drive
    //% weight=100 blockGap=8
    //% block="move %direction|at speed %speed"
    //% speed.min=0 speed.max=100
    export function move(direction: DriveDirections, speed: number): void {
        if (initalised == false) {
            setup()
        }
        switch (direction)
        {
            case DriveDirections.Forward:
                motorOn(Motors.MotorLeft, MotorDirection.Forward, speed)
                motorOn(Motors.MotorRight,MotorDirection.Forward, speed)
            break
            case DriveDirections.Reverse:
                motorOn(Motors.MotorLeft, MotorDirection.Reverse, speed)
                motorOn(Motors.MotorRight,MotorDirection.Reverse, speed)
            break
            case DriveDirections.Left:
                motorOn(Motors.MotorLeft,MotorDirection.Forward, (speed/turnTightness))
                motorOn(Motors.MotorRight, MotorDirection.Forward, speed)
            break
            case DriveDirections.Right:
                motorOn(Motors.MotorLeft,MotorDirection.Forward, speed)
                motorOn(Motors.MotorRight, MotorDirection.Forward, (speed/turnTightness))
            break
            default: //just in case. Should never get here
                motorOff(Motors.MotorLeft)
                motorOff(Motors.MotorRight)
            break
        }
    }

    /**
     * Tunrs on the spot in the direction requested.
     * @param direction Direction to spin in
     * @param speed How fast to go (0-100)
     */
    //% subcategory=Motors
    //% group="Drive"
    //% blockId=kitronik_move_motor_spin
    //% weight=95 blockGap=8
    //% block="spin %direction|at speed %speed"
    //% speed.min=0, speed.max=100
    export function spin(direction: SpinDirections, speed: number): void {
         if (initalised == false) {
            setup()
        }
        switch (direction)
        {
            case SpinDirections.Left:
                 motorOn(Motors.MotorLeft, MotorDirection.Reverse, speed)
                 motorOn(Motors.MotorRight,MotorDirection.Forward, speed)
            break
            case SpinDirections.Right:
                 motorOn(Motors.MotorLeft, MotorDirection.Forward, speed)
                 motorOn(Motors.MotorRight,MotorDirection.Reverse, speed)
            break
            default: //just in case. Should never get here
                motorOff(Motors.MotorLeft)
                motorOff(Motors.MotorRight)
            break
        }
    }
    
    /**
     * Stops the :MOVE motor driving
     */
    //% subcategory=Motors
    //% group="Drive"
    //% blockId=kitronik_move_motor_stop
    //% weight=90 blockGap=8
    //% block="stop"
    export function stop(): void {
         if (initalised == false) {
            setup()
        }
        motorOff(Motors.MotorLeft)
        motorOff(Motors.MotorRight)
    }

    /**
     * To help the :MOVE motor drive in a straight line you can bias the motors.
     * @param balance number between 0 and 10 to help balance the motor speed
     */
    //% subcategory=Motors
    //% group="Setup"
    //% blockId=kitronik_move_motor_motor_balance
    //% weight=85 blockGap=8
    //% block="bias to %direction by %balance"
    //% balance.min=0 balance.max=10
    export function motorBalance(direction: SpinDirections, balance: number): void {
        if (balance < 0) {
            balance = 0
        }
        else if (balance > 10) {
            balance = 10
        }
        leftMotorBias = 0
        rightMotorBias = 0
        switch (direction)
        {
            case SpinDirections.Left:
            leftMotorBias = Math.round(balance*1.75)
            break
            case SpinDirections.Right:
            rightMotorBias = Math.round(balance*1.75)
        }
    }

     /**
     * Changes how tight the :MOVE motor turns when move block is used with Left or Right
     * @param radius the tightness of the turn
     */
    //% subcategory=Motors
    //% group="Setup"
    //% blockId=kitronik_move_motor_motor_turn_radius
    //% weight=80 blockGap=8
    //% block="set turn radius %radius"
    export function turnRadius(radius:TurnRadii): void {
        switch (radius)
        {
            case TurnRadii.Tight:
                turnTightness = 8
            break
            case TurnRadii.Wide:
                turnTightness= 1.6
            break
            case TurnRadii.Standard: 
                turnTightness = 3
            break
        }
    }
    
    //Here for the more advanced user - this function sets the divider for the speed of the slower wheel in a turn.
    //setting it to 2 will result in the inner wheel of the turn running at 1/2 the speed of the outer wheel.
    // No checking of what the value is - If you are using this it is expected you know what you are doing
    export function setTurnRadius(radiusDivider:number)
    {
        turnTightness = radiusDivider
    }
    
     /**
     * Sets the requested motor running in chosen direction at a set speed.
     * If setup is not complete, calls the initialisation routine.
     * @param motor which motor to turn on
     * @param dir which direction to go
     * @param speed how fast to spin the motor
     */
    //% subcategory=Motors
    //% group="Motor Control"
    //% blockId=kitronik_move_motor_motor_on
    //% block="turn %motor|motor on direction %dir|speed %speed"
    //% weight=75 blockGap=8
    //% speed.min=0 speed.max=100
    export function motorOn(motor: Motors, dir: MotorDirection, speed: number): void { 
        if (initalised == false) {
            setup()
        }
        /*convert 0-100 to 0-255 by a simple multiple by 2.55*/
        let outputVal = Math.round(speed*2.55)
        if (outputVal > 255){ 
            outputVal = 255 
        }

        // Depending on board version, motors are driven in different ways: V1.0 & V1.3 - PCA9632, V3.1 - WS2811
        if (moveMotorVersion == 31) {
            let motorOnbuf = pins.createBuffer(6)
            //The jerk message gives the motor a 'shove' at full power to aid starting on lower pwm ratios
            let motorJerkBuf = pins.createBuffer(6)
            switch (motor) {
                case Motors.MotorRight:
                    switch (dir) {
                        case MotorDirection.Forward:
                            motorBuf[0] = outputVal - rightMotorBias
                            motorBuf[1] = 0
                            motorJerkBuf[0] = 255
                            motorJerkBuf[1] = 0
                            break
                        case MotorDirection.Reverse:
                            motorBuf[0] = 0
                            motorBuf[1] = outputVal - rightMotorBias
                            motorJerkBuf[0] = 0
                            motorJerkBuf[1] = 255
                            break
                    }
                    if (outputVal == 0) {
                        motorBuf[2] = 255
                    }
                    else {
                        motorBuf[2] = 0
                    }
                    break
                case Motors.MotorLeft:
                    switch (dir) {
                        case MotorDirection.Forward:
                            motorBuf[3] = 0
                            motorBuf[4] = outputVal - leftMotorBias
                            motorJerkBuf[3] = 0
                            motorJerkBuf[4] = 255
                            break
                        case MotorDirection.Reverse:
                            motorBuf[3] = outputVal - leftMotorBias
                            motorBuf[4] = 0
                            motorJerkBuf[3] = 255
                            motorJerkBuf[4] = 0
                            break
                    }
                    if (outputVal == 0) {
                        motorBuf[5] = 255
                    }
                    else {
                        motorBuf[5] = 0
                    }
                    break
                default:
                //Stop - something has gone wrong
            }

            // Send the data to the WS2811 ICs
            if (outputVal != 0) {
                //Kitronik_WS2811.sendBuffer(motorJerkBuf, motorPin)
                basic.pause(1)
            }
            latestMotorBuf = motorBuf
            //Kitronik_WS2811.sendBuffer(motorBuf, motorPin)
        }
        else {
            let motorOnbuf1 = pins.createBuffer(5)
            //The jerk message gives the motor a 'shove' at full power to aid starting on lower pwm ratios
            let motorJerkBuf1 = pins.createBuffer(5)

            motorOnbuf1[0] = 0xA2
            motorJerkBuf1[0] = 0xA2
            switch (motor) {
                case Motors.MotorRight:
                switch (dir) {
                    case MotorDirection.Forward:
                        PWMReg1 = 0
                        PWMReg2 = outputVal - rightMotorBias
                        motorJerkBuf1[1] = 0
                        motorJerkBuf1[2] = 0xFF
                        break
                    case MotorDirection.Reverse:
                        PWMReg1 =  outputVal - rightMotorBias
                        PWMReg2 = 0
                        motorJerkBuf1[1] = 0xFF
                        motorJerkBuf1[2] = 0
                        break
                }
                    break
                case Motors.MotorLeft:
                switch (dir) {
                    case MotorDirection.Forward:
                        PWMReg3 = outputVal - leftMotorBias
                        PWMReg4 = 0
                        motorJerkBuf1[3] = 0xFF
                        motorJerkBuf1[4] = 0
                        break
                    case MotorDirection.Reverse:
                        PWMReg3 = 0
                        PWMReg4 = outputVal - leftMotorBias
                        motorJerkBuf1[3] = 0
                        motorJerkBuf1[4] = 0xFF
                        break
                }
                    break
                default:
                //Stop - something has gone wrong
            }

            motorOnbuf1[1] = PWMReg1
            motorOnbuf1[2] = PWMReg2
            motorOnbuf1[3] = PWMReg3
            motorOnbuf1[4] = PWMReg4

            //At this point we have updated the required PWM Registers, so write it to the Chip
            pins.i2cWriteBuffer(CHIP_ADDR, motorJerkBuf1, false)
            basic.pause(1) //let the motor start before throttling them to the lower level
            pins.i2cWriteBuffer(CHIP_ADDR, motorOnbuf1, false)
        }
    }

    /**
     * Turns off the specified motor.
     * @param motor which motor to turn off
     */
    //% subcategory=Motors
    //% group="Motor Control"
    //% blockId=kitronik_move_motor_motor_off
    //% weight=70 blockGap=8
    //% block="turn off %motor| motor"
    export function motorOff(motor: Motors): void {
        if (initalised == false) {
            setup()
        }

        // Depending on board version, motors are driven in different ways: V1.0 & V1.3 - PCA9632, V3.1 - WS2811
        if (moveMotorVersion == 31) {
            let motorOnbuf = pins.createBuffer(6)
            switch (motor) {
                case Motors.MotorRight:
                    motorBuf[0] = 0
                    motorBuf[1] = 0
                    motorBuf[2] = 255 // Turn on the brake light
                    break
                case Motors.MotorLeft:
                    motorBuf[3] = 0
                    motorBuf[4] = 0
                    motorBuf[5] = 255 // Turn on the brake light
                    break
                default:
                //Stop - something has gone wrong
            }
            
            // Send the data to the WS2811 ICs
            //Kitronik_WS2811.sendBuffer(motorBuf, motorPin)
        }
        else {
            let motorOnbuf1 = pins.createBuffer(5)
            motorOnbuf1[0] = 0xA2
            switch (motor) {
                case Motors.MotorRight:
                    PWMReg1 = 0
                    PWMReg2 = 0
                    break
                case Motors.MotorLeft:
                    PWMReg3 = 0
                    PWMReg4 = 0
                    break
            }
            motorOnbuf1[1] = PWMReg1
            motorOnbuf1[2] = PWMReg2
            motorOnbuf1[3] = PWMReg3
            motorOnbuf1[4] = PWMReg4
            //At this point we have updated the required PWM Registers, so write it to the Chip
            pins.i2cWriteBuffer(CHIP_ADDR, motorOnbuf1, false)
        }
    }

    //////////////
    //  SOUNDS  //
    //////////////

    /**
     * Beep horn 
     */
    //% subcategory=Sounds
    //% blockId=kitronik_move_motor_horn
    //% weight=95 blockGap=8
    //%block="beep the horn"
    export function beepHorn(): void {
            music.playTone(185, music.beat(BeatFraction.Quarter))
            basic.pause(75)
    }

    /**
    * Turns on and off the siren that plays in the background.
    * @param OnOffSelection which selects the status of the siren being on or off
    */
    //% subcategory=Sounds
    //% blockId=kitronik_move_motor_siren
    //% weight=90 blockGap=8
    //% block="turn siren %siren"
    export function soundSiren(siren: OnOffSelection): void {
        if (siren == OnOffSelection.On) {
            sirenOn = true
            control.inBackground(() => {
            while (sirenOn) {
                music.playTone(370, music.beat(BeatFraction.Half))
                basic.pause(75)
                music.playTone(262, music.beat(BeatFraction.Half))
                basic.pause(75)
            }

        })
        }
        else {
            sirenOn = false
            music.stopMelody(MelodyStopOptions.Background)
        }
    }
    
    
    ////////////////////
    //  SERVO & PINS  //
    ////////////////////
    /**
    * Read the digital value of the pin selected.
    * @param PinSelection is the list of pins avaible to choose from.
    */
    //% subcategory=Pins
    //% blockId=kitronik_move_motor_digital_read
    //% weight=90 blockGap=8
    //% block="digital read pin %pin"
    export function readDigitalPin(pin: PinSelection): number {
        if (pin == PinSelection.P15) {
            return pins.digitalReadPin(DigitalPin.P15)
        }
        else{
            return pins.digitalReadPin(DigitalPin.P16)
        }
    }
    
    /**
    * Write the digital value of the pin selected.
    * @param PinSelection is the list of pins avaible to choose from.
    */
    //% subcategory=Pins
    //% blockId=kitronik_move_motor_digital_write
    //% weight=90 blockGap=8
    //% block="digital write pin %pin to %value"
    //% value.min=0 value.max=1
    export function writeDigitalPin(pin: PinSelection, value: number): void {
        if (pin == PinSelection.P15) {
            pins.digitalWritePin(DigitalPin.P15, value)
        }
        else{
            pins.digitalWritePin(DigitalPin.P16, value)
        }
    }

    /**
    * Write the analog value of the pin selected.
    * @param PinSelection is the list of pins avaible to choose from.
    */
    //% subcategory=Pins
    //% blockId=kitronik_move_motor_analog_write
    //% weight=90 blockGap=8
    //% block="analog write pin %pin to %value"
    //% value.min=0 value.max=1023
    export function writeAnalogPin(pin: PinSelection, value: number): void {
        if (pin == PinSelection.P15) {
            pins.analogWritePin(AnalogPin.P15, value)
        }
        else{
            pins.analogWritePin(AnalogPin.P16, value)
        }
    }

    /**
    * Write the servo angle of the pin selected for driving a servo.
    * @param PinSelection is the list of pins avaible to choose from.
    */
    //% subcategory=Pins
    //% blockId=kitronik_move_motor_servo_write
    //% weight=90 blockGap=8
    //% block="write %servo to %angle"
    //% angle.min=0 angle.max=180
    export function writeServoPin(servo: ServoSelection, angle: number): void {
        if (servo == ServoSelection.servo1) {
            pins.servoWritePin(AnalogPin.P15, angle)
        }
        else{
            pins.servoWritePin(AnalogPin.P16, angle)
        }
    }
}
