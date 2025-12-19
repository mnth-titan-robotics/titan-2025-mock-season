from wpilib import AddressableLED, LEDPattern


class LightsController:
    def __init__(self):
        self._controller = AddressableLED(1)

        # led strip things
        total_strips = 1
        leds_per_strip = 144
        self._controller.setLength(total_strips * leds_per_strip)
