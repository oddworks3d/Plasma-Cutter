# Coded by Tea S.
# 2022

# @formatter:on
from random import randint

from collections import namedtuple
from machine import Pin, PWM, freq
import utime
import neopixel
import json

# Overlock pico to max cpu freq
# freq(180000000)

# Hardcoded config file for if it doesnt exist on the pico
config = {"buttons":
    {
        "trigger1": 1,
        "trigger2": 2,
        "mag": 3, },
    "lights": {
        'barrel-light-1': {'length': 1, 'pin': 4, 'flicker': False},
        'barrel-light-2': {'length': 1, 'pin': 6, 'flicker': False},
        'barrel-light-3': {'length': 1, 'pin': 7, 'flicker': False},
        'ammo-count': {'length': 2, 'pin': 8, 'flicker': False}
    },
    "colors": {
        'heating': [[255, 80, 0], [0, 0, 0]],
        'normal': [[72, 209, 104], [255, 255, 255]],
        'hot': [[225, 0, 0], [1, 1, 1]],
        'flash': [[200, 255, 255], [15, 15, 15]],
        'yellow': [[225, 255, 0], [15, 15, 15]],
        'red': [[255, 0, 0], [15, 15, 15]],
    },
    "servos": {"barrel-rotation": {"invert": False, "start": 1, "speed": 50, "curpos": 2, "end": 180, "pin": 10},
               "barrel": {"invert": True, "start": 1, "speed": 50, "curpos": 45, "end": 180, "pin": 12}},
    "state": "Main",
    "maxAmmo": "6"}

# Load config file from system (it holds all the servo settings and current position so the servos dont judder when it turns back on)
if "config" in os.listdir():
    try:
        file = open("config", "r")
        readConfig = json.loads(file.readline())
        if readConfig != config:
            print("Config file changed, recreating")
            file = open("config", "wb")
            file.write(json.dumps(config))
        else:
            config = readConfig
        file.close()
    except ValueError:
        # Problem reading config file, recreate it using the above hardcoded config file
        print("Unable to read config file...Recreating")
        file = open("config", "wb")
        file.write(json.dumps(config))
        file.close()
else:
    # Create config file if it doesn't exist
    file = open("config", "wb")
    file.write(json.dumps(config))
    file.close()


def saveSettings(wherefrom=""):
    print("save - " + wherefrom)
    print(config)
    file = open("config", "wb")
    file.write(json.dumps(config))
    file.close()


print(config)


# ===========================START OF CLASSES ==================================


# Class for controlling and setting servo settings, pass in a config file with all the settings of the servo
class Servo:
    def __init__(self, config):
        self.speed = config['speed']
        self.posMin = config['start']
        self.posMax = config['end']
        self.pin = config['pin']
        self.invert = config['invert']
        self.config = config
        self.targetAngle = config['curpos']
        self.angle = config['curpos']
        self.pwm = PWM(Pin(self.pin, Pin.OUT))
        self.pwm.freq(50)

    def setPos(self, angle, immediate=False):
        """
        Set the position of the servo motor

        Parameters:
            angle (float): The desired angle for the servo motor in degrees from 0 to 180.
            immediate (bool, optional): If True, the servo motor will immediately move to the specified angle. If False, the servo motor will move to the specified angle gradually. Defaults to False.

        Returns:
            None
        """
        if not self.isMoving():
            if round(angle) != round(self.angle):
                if angle > self.posMax:
                    angle = self.posMax
                elif angle < self.posMin:
                    angle = self.posMin
                if immediate:
                    self.angle = angle
                    self.targetAngle = angle
                    self._moveServo(angle)
                else:
                    self.targetAngle = angle
                # save angle to config file here

    def setMin(self, min):
        """
        Sets the minimum value for the position and updates the position accordingly.

        Parameters:
            min (int): The minimum value for the position.

        Returns:
            None
        """
        self.posMin = min
        self.setPos(min)

    def setMax(self, max):
        """
        Set the maximum value for the position.

        Parameters:
            max (int): The maximum value for the position.

        Returns:
            None
        """
        self.posMax = max
        self.setPos(max)

    def setSpeed(self, speed):
        """
        Set the speed of the object.

        Parameters:
            speed (int): The desired speed of the object.

        Returns:
            None
        """
        if speed != self.speed:
            self.speed = speed

    def getSpeed(self):
        """
        Get the speed of the object.

        Returns:
            The speed of the object.
        """
        return self.speed

    def setInvert(self, invert):
        self.invert = invert

    def update(self):
        if self.isMoving():
            print(self.config, self.angle)
            if self.angle < self.targetAngle:
                self.angle += (self.speed / 100)
            elif self.angle > self.targetAngle:
                self.angle -= (self.speed / 100)
            self._moveServo(int(self.angle))

    def _moveServo(self, degrees):
        # limit degrees between 0 and 180
        if degrees > 180:
            degrees = 180
        if degrees < 0:
            degrees = 0
        if self.invert:
            degrees = (180 + 0) - degrees
        # set max and min duty
        maxDuty = 9000
        minDuty = 1500
        # new duty is between min and max duty in proportion to its value
        newDuty = minDuty + (maxDuty - minDuty) * (degrees / 180)
        # servo PWM value is set
        self.pwm.duty_u16(int(newDuty))

    def isMoving(self):
        if round(self.angle) == round(self.targetAngle):
            return False
        else:
            return True

    def open(self, immediate=False):
        self.setPos(self.posMax, immediate=immediate)

    def close(self, immediate=False):
        self.setPos(self.posMin, immediate=immediate)
        # Need to improve this, only works if the servo was set using open() or close()

    def toggle(self):
        if round(self.angle) == round(self.posMax):
            self.close()
        if round(self.angle) == round(self.posMin):
            self.open()

    def saveCurPos(self):
        self.config['curpos'] = self.targetAngle
        saveSettings("savecurpos")


# Pixel class based on RGB tuples, capable of animation from one colour to another, adjusting brightness as a whole and enabling a cool flicker effect (possibly more in the future)
class Pixel:
    def __init__(self, speed, brightness, color, flicker=True):
        # Normal Flicker
        self.targetBrightness = 0
        self.brightness = 0

        self.color = list(color[0])
        self.flashColor = list(color[1])

        self.targetColor = list(color[0])
        self.targetFlashColor = list(color[1])

        self.randomFlicker = randint(int(1 * speed), int(20 * speed))
        self.speed = speed
        self.randomFlickerCount = 0
        self.randomFlickerOn = 0
        self.brightnessNormal = 0
        self.brightnessFlicker = 0
        self.animationSpeed = 100
        self.isFlicker = flicker
        self.animating = False
        self.setBrightness(brightness)
        self.brightnessTimer = Timer(0, 1, 1)
        self.colorTimers = [Timer(0, 1, 1), Timer(0, 1, 1), Timer(0, 1, 1)]

    def getPixelState(self):
        self.randomFlickerCount += 1
        self.randomFlickerOn -= 1
        if self.randomFlickerCount >= self.randomFlicker:
            self.randomFlicker = randint(
                int(1 * self.speed), int(20 * self.speed))
            self.randomFlickerOn = 1
            self.randomFlickerCount = 0
        if self.randomFlickerOn > 0 and self.isFlicker:
            return tuple([int(x * self.brightness / 100) for x in self.flashColor])
        else:
            return tuple([int(x * self.brightness / 100) for x in self.color])

    def update(self):
        # Handle colour animation
        if self.color != self.targetColor:
            for i in range(len(self.color)):
                if self.color[i] > self.targetColor[i]:
                    self.color[i] = max(0, self.color[i] - 1)
                elif self.color[i] < self.targetColor[i]:
                    self.color[i] = min(255, self.color[i] + 1)
                if self.flashColor[i] > self.targetFlashColor[i]:
                    self.flashColor[i] = max(self.targetFlashColor[i], self.flashColor[i] - 1)
                elif self.flashColor[i] < self.targetFlashColor[i]:
                    self.flashColor[i] = min(self.targetFlashColor[i], self.flashColor[i] + 1)
        if self.brightness != self.targetBrightness:
            if self.brightnessTimer.update(True):
                if self.brightness > self.targetBrightness:
                    self.brightness = self.brightness - 1
                if self.brightness < self.targetBrightness:
                    self.brightness = self.brightness + 1

                if self.brightness > 100:
                    self.brightness = 100
                elif self.brightness < 0:
                    self.brightness = 0
                self.brightnessTimer.reset()

        # Handle random flickering effect
        return self.getPixelState()

    # Get rgb tuple for writing to neopixels

    def getRgb(self):
        return self.rgbCurrent

    def isAnimating(self):
        return self.animatng

    # Set the color to animate to

    def animateColor(self, color, speed=0):
        if speed != 0:
            self.animationSpeed = speed
        if self.targetColor != color:
            self.targetColor = list(color[0])
            self.targetFlashColor = list(color[1])

    def setColor(self, color):
        self.targetColor = list(color[0])
        self.targetFlashColor = list(color[1])
        self.color = list(color[0])
        self.flashColor = list(color[1])

    def setAnimationSpeed(self, speed):
        self.animationSpeed = speed

    def setBrightness(self, brightness):
        if self.brightness != brightness:
            if brightness > 100:
                brightness = 100
            elif brightness <= 0:
                brightness = 0
            self.brightness = brightness
            self.targetBrightness = brightness

    def animateBrightness(self, brightness, speed=0):
        if speed != 0:
            self.animationSpeed = speed
        if self.targetBrightness != brightness:
            self.targetBrightness = brightness
            if (brightness - self.brightness) != 0:
                delta = abs(self.animationSpeed / (brightness - self.brightness))
                self.brightnessTimer = Timer(0, delta, 1)
            else:
                self.brightnessTimer = Timer(0, 1, 1)

    def setFlicker(self, flicker):
        self.isFlicker = flicker


# Class for handling neolight lighting (just holds individual pixels and updates them all the logic happens in the pixel class above)
class NeoLight:
    def __init__(self, config):
        self.config = config
        self.brightness = 0
        self.pin = config['pin']
        self.cooldown = False
        self.speed = 1
        self.length = config['length']
        self.np = neopixel.NeoPixel(Pin(config['pin']), config['length'])
        self.pixels = []
        self.color = [[225, 0, 0], [15, 15, 15]]
        self.cooldown = False
        self.flicker = config['flicker']
        for i in range(len(self.np)):
            pixel = Pixel(self.speed, self.brightness,
                          self.color, self.flicker)
            self.pixels.append(pixel)

    def update(self):
        for index, pixel in enumerate(self.pixels):
            self.np[index] = pixel.update()
        self.np.write()

    def animateColor(self, color, speed=0):
        if color != self.color:
            for pixel in self.pixels:
                if speed > 0:
                    pixel.animateColor(color, speed)
                else:
                    pixel.animateColor(color)
            self.color = color

    def animateColorRange(self, color, start, end, speed=0):
        if color != self.color:
            for index, pixel in enumerate(self.pixels):
                if start <= index <= end:
                    if speed > 0:
                        pixel.animateColor(color, speed)
                    else:
                        pixel.animateColor(color)
            self.color = color

    def setColor(self, color):
        for index, pixel in enumerate(self.pixels):
            if index < self.length:
                pixel.setColor(color)
                self.color = color

    def setColorRange(self, color, start, end):
        for index, pixel in enumerate(self.pixels):
            if start <= index <= end:
                pixel.setColor(color)
                self.color = color

    def setPixelColor(self, color, pixel):
        self.pixels[pixel].setColor(color)

    def setBrightness(self, brightness, animate=False, speed=0):
        if self.brightness != brightness:
            if brightness > 100:
                brightness = 100
            elif brightness < 0:
                brightness = 0
            for index, pixel in enumerate(self.pixels):
                if index < self.length:
                    if animate:
                        pixel.animateBrightness(brightness, speed)
                    else:
                        pixel.setBrightness(brightness)
            self.brightness = brightness

    def setBrightnessRange(self, brightness, start, end, animate=False, speed=0):
        """
        Sets the brightness of the NeoPixel strip between start and end
        :param brightness: Level of brightness from 0-100
        :param start: Start pixel
        :param end: End pixel
        :param animate: Whether to animate the brightness change or not
        """
        if brightness > 100:
            brightness = 100
        elif brightness < 0:
            brightness = 0
        for index, pixel in enumerate(self.pixels):
            if start <= index <= end:
                if pixel.brightness != brightness:
                    if animate:
                        pixel.animateBrightness(brightness, speed)
                    else:
                        pixel.setBrightness(brightness)
            else:
                # pixel.setBrightness(0)
                pass

    def setFlicker(self, flicker):
        self.flicker = flicker
        for index, pixel in enumerate(self.pixels):
            pixel.setFlicker(flicker)

    def setLength(self, length):
        self.length = length


# Class for handling input buttons, you can set if it should fire off once or press and hold to continually output true after a predefined timeout.
class Button:
    """
    Initalizes a new button

    Args:
        pin (int): The pin number.
        single (bool, optional): Indicates whether the button is single-click only. Defaults to False.

    Returns:
        None
    """

    def __init__(self, pin, single=False):
        self.single = single
        self.pin = pin
        self.lock = False
        self.button = Pin(pin, Pin.IN, Pin.PULL_DOWN)
        self.held = False

    def getState(self, single=True):
        """
        Gets the state of the button.

        Parameters:
            single (bool): Whether the button is single-click only (returns true once)

        Returns:
            bool: The state of the button.
        """
        if self.button.value():
            self.held = True
            if not self.lock:
                self.lock = True
                return True
            else:
                if not single:
                    return True
                return False
        else:
            self.lock = False
            self.held = False
            return False

    def getHeld(self):
        return self.held

    def reset(self):
        self.lock = True


# Basic timer class to count down things and call a method when it reaches its goal
class Timer:
    def __init__(self, start, end, interval, functions=[]):
        self.end = end
        self.start = start
        self.current = start
        self.interval = interval
        self.functions = functions

    def update(self, do):
        if do:
            if self.interval > 0:
                if self.current <= self.end:
                    self.current = self.current + self.interval
            else:
                if self.current >= self.start:
                    self.current = self.current + self.interval
        if self.current >= self.end:
            for function in self.functions:
                if callable(function):
                    function()
            return True
        else:
            return False

    def reset(self):
        self.current = self.start

    def getState(self):
        return self.current

    def setStartEnd(self, start, end):
        self.start = start
        self.end = end


buttons = {}
# Initialize all the buttons
for button in config['buttons']:
    buttons[button] = Button(config['buttons'][button], True)


# =============================== METHODS ==================================
def set_pixel_line_gradient(pixel1, pixel2, left_rgb, right_rgb, light):
    if pixel2 - pixel1 == 0:
        return
    right_pixel = max(pixel1, pixel2)
    left_pixel = min(pixel1, pixel2)

    r_diff = right_rgb[0] - left_rgb[0]
    g_diff = right_rgb[1] - left_rgb[1]
    b_diff = right_rgb[2] - left_rgb[2]
    for i in range(right_pixel - left_pixel):
        fraction = i / (right_pixel - left_pixel)
        red = round(r_diff * fraction + left_rgb[0])
        green = round(g_diff * fraction + left_rgb[1])
        blue = round(b_diff * fraction + left_rgb[2])
        light.setPixelColor([[red, green, blue], [0, 0, 0]], left_pixel + i)


def paintLines(frame):
    if frame > 0:
        if frame < 20:
            set_pixel_line_gradient(0, max(frame, 0), [255, 80 - (frame * 2), 0], [
                0, 255, 150], frontLights['heat-left'])
            set_pixel_line_gradient(0, max(frame, 0), [255, 80 - (frame * 2), 0], [
                0, 255, 150], frontLights['heat-right'])
        else:
            set_pixel_line_gradient(
                max(frame - 20, 0), 24, [255, 80 - (frame * 2), 0], [0, 255, 150], frontLights['heat-left'])
            set_pixel_line_gradient(
                max(frame - 20, 0), 24, [255, 80 - (frame * 2), 0], [0, 255, 150], frontLights['heat-right'])
    else:
        frontLights['heat-left'].setColor(config['colors']['normal'])
        frontLights['heat-right'].setColor(config['colors']['normal'])


def changeState(toChange):
    global state
    state = toChange


# Initialize all the NeoPixels
frontLights = {}
ammoCountLight = NeoLight(config['lights']['ammo-count'])
ammoCountLight.setColor(config['colors']['normal'])

for light in config['lights']:
    if light != 'ammo-count':
        frontLights[light] = NeoLight(config['lights'][light])

# Update servos once before continuing (sets them to the same position as in the config file)
barrelRotation = Servo(config['servos']['barrel-rotation'])
barrel = Servo(config['servos']['barrel'])
barrelRotation.setPos(38, True)
barrelRotation.update()
barrel.update()

barrel.setPos(50, True)
utime.sleep(2)
barrel.setPos(90, True)


class State():
    def __init__(self, saveState=False):
        self.saveState = saveState

    def enter(self, sm):
        pass

    def update(self, sm):
        pass

    def exit(self, sm):
        pass


# ========= States ===============
class Main(State):
    def __init__(self):
        self.printTimer = Timer(0, 200, 1)
        super().__init__()

    def enter(self, sm):
        for light in frontLights:
            frontLights[light].setColor(config['colors']['normal'])
            frontLights[light].setBrightness(sm.normalBrightness, True, 100)
        pass

    def update(self, sm):
        if sm.ammo <= 0:
            sm.changeState(Closing())
        if buttons['trigger1'].getState():
            sm.changeState(Firing())


class Firing(State):

    def __init__(self):
        super().__init__()
        self.flashTimer = Timer(0, 10, 1)

    def enter(self, sm):
        for light in frontLights:
            frontLights[light].setBrightness(100)
            frontLights[light].setColor(config['colors']['flash'])

    def update(self, sm):
        if self.flashTimer.update(True):
            for light in frontLights:
                frontLights[light].setBrightness(sm.normalBrightness)
                frontLights[light].setColor(config['colors']['normal'])
            if buttons['trigger2'].getState():
                if sm.ammo <= 0:
                    sm.changeState(Closing())
                else:
                    if not sm.barrelOpen:
                        sm.changeState(Opening())
                    else:
                        sm.changeState(Closing())
            else:
                sm.changeState(Main())

    def exit(self, sm):
        sm.ammo -= 1
        pass


class Opening(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        barrel.setPos(113)
        sm.barrelOpen = True
        pass

    def update(self, sm):
        if not barrel.isMoving():
            barrelRotation.setPos(117)
            if not barrelRotation.isMoving():
                sm.changeState(Main())

    def exit(self, sm):
        pass


class Closing(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        barrelRotation.setPos(38)
        sm.barrelOpen = False
        pass

    def update(self, sm):
        if not barrelRotation.isMoving():
            barrel.setPos(90)
            if not barrel.isMoving():
                if sm.ammo <= 0:
                    sm.changeState(NoAmmo())
                else:
                    sm.changeState(Main())
        pass

    def exit(self, sm):
        pass


class NoAmmo(State):
    def __init__(self):
        super().__init__()

    def enter(self, sm):
        # Turn off front lights
        for light in frontLights:
            frontLights[light].setBrightness(0, True, 100)
        barrel.setPos(0)

    def update(self, sm):
        if buttons['mag'].getState():
            sm.changeState(Main())
        pass

    def exit(self, sm):
        sm.ammo = 6
        barrel.setPos(90)
        for light in frontLights:
            frontLights[light].setBrightness(sm.normalBrightness, True, 100)
        pass


class StateMachine:
    def __init__(self, starting):
        self.currentState = None
        # Settings
        self.normalBrightness = 15
        self.ammo = int(config['maxAmmo'])
        self.changeState(starting)
        self.barrelOpen = False
        self.printTimer = Timer(0, 1, 1)

    def changeState(self, state):
        """
        Change the current state of the state machine to the specified state.
        if state has variable called saveState and it is true, saves state to config file

        Parameters:
            state (State): The new state to be set.

        Returns:
            None
        """
        if self.currentState:
            self.currentState.exit(self)
            # Save new state on exiting old state
            if self.currentState.saveState:
                config['state'] = type(state).__name__
                saveSettings()

        self.currentState = state

        if self.currentState:
            self.currentState.enter(self)
            if self.currentState.saveState:
                config['state'] = type(state).__name__
                saveSettings()

    def getCurrentState(self):
        return self.currentState

    def update(self):
        self.currentState.update(self)
        if barrel.isMoving():
            barrel.update()
        if barrelRotation.isMoving():
            barrelRotation.update()
        for light in frontLights:
            frontLights[light].update()
        self.checkAmmo()
        ammoCountLight.update()

    def checkAmmo(self):
        if int(self.ammo) == 6:
            ammoCountLight.setBrightness(100, True, 100)
            ammoCountLight.setColor(config['colors']['normal'])
        elif int(self.ammo) == 5:
            ammoCountLight.setBrightnessRange(50, 1, 1, True)
        elif int(self.ammo) == 4:
            ammoCountLight.setBrightnessRange(15, 1, 1, True)
        elif int(self.ammo) == 3:
            ammoCountLight.setBrightnessRange(0, 1, 1, True)
        elif int(self.ammo) == 2:
            ammoCountLight.animateColorRange(config['colors']['yellow'], 0, 0, 1)
        elif int(self.ammo) == 1:
            ammoCountLight.animateColorRange(config['colors']['red'], 0, 0, 1)
        elif int(self.ammo) <= 0:
            self.ammo = 0
            ammoCountLight.setBrightness(0, True, 100)

    def printSlowly(self, msgs=[]):
        if self.printTimer.update(True):
            for msg in msgs:
                print(msg)
            self.printTimer.reset()


# Load into previously saved state:
# Only if one of the "safe" states
if config['state'] == 'Main' or config['state'] == 'Exploded':
    state = config['state']
else:
    state = 'Main'
constructor = globals()[state]

mainLogic = StateMachine(constructor())

# MAIN LOOP
loopstart = utime.time()

while True:
    mainLogic.update()
