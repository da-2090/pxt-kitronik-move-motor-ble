# pxt-kitronik-move-motor

Minimal extension for Kitronik :MOVE Motor focused on:

- motors
- ultrasonic distance sensor
- bluetooth compatibility

Lights and sound blocks are intentionally removed.

## Use this extension

Add this repository as an extension in MakeCode:

- open https://makecode.microbit.org/
- create a new project
- open Extensions
- import this repository URL

## Available blocks

### Ultrasonic

```blocks
basic.showNumber(Kitronik_Move_Motor.measure())
```

### Drive

```blocks
Kitronik_Move_Motor.move(Kitronik_Move_Motor.DriveDirections.Forward, 50)
basic.pause(500)
Kitronik_Move_Motor.stop()
```

### Spin

```blocks
Kitronik_Move_Motor.spin(Kitronik_Move_Motor.SpinDirections.Left, 35)
```

### Direct motor control

```blocks
Kitronik_Move_Motor.motorOn(Kitronik_Move_Motor.Motors.MotorLeft, Kitronik_Move_Motor.MotorDirection.Forward, 100)
basic.pause(300)
Kitronik_Move_Motor.motorOff(Kitronik_Move_Motor.Motors.MotorLeft)
```

## Notes

- V3.1 boards use internal WS2811 motor signalling.
- This package keeps that driver local to avoid bluetooth being disabled by an external dependency.
- LED light control is not exposed.

## License

MIT
