import pyb
class encoderDriver:
    """
    @brief Class for reading encoder values using a timer.
    @details This class sets up and reads values from an encoder connected to arbitrary pins.

    @param timer: The timer object to use for reading.
    @param channel_a_pin: Pin for channel A of the encoder.
    @param channel_b_pin: Pin for channel B of the encoder.
    @param max_count: The maximum count value for the timer.
    """

    def __init__(self, timer, channel_a_pin, channel_b_pin, max_count):
        # Reassign the arguments to the object. 
        self.timer = timer
        self.channel_a_pin = channel_a_pin
        self.channel_b_pin = channel_b_pin
        self.max_count = max_count # Auto reload value

        # Configure the timer channels for encoder counting (ENC_AB mode)
        # This part was in the lab manual.
        self.timer.channel(1, pin=channel_a_pin, mode=pyb.Timer.ENC_AB)
        self.timer.channel(2, pin=channel_b_pin, mode=pyb.Timer.ENC_AB)

        # Initialize variables to keep track of encoder count and position
        self.encoder_count = 0 #from the last timer channel interrupt
        self.delta = 0 #change in position between now and last timer channel interrupt
        self.current_count = 0 # overflows
        self.position = 0 #encoder ticks, doesn't overflow
        self.half_max_count = self.max_count / 2 
        self.neg_half_max_count = -self.half_max_count

    def update(self):
        """
        @brief Updates the recorded position of the encoder.
        @details This method should be called regularly to update the position.
        It computes the change in encoder count since the last update and adds it to the position.
        """
        # Read the encoder count
        # This is result of the XOR of channels 1 and 2 of the user specified timer module.
        self.current_count = self.timer.counter()

        # Change in position (delta)
        self.delta = self.current_count - self.encoder_count

        # Handle timer overflow
        # Should one of these +1's be negative? --Max 
        # No, it deals with it.
        if self.delta > self.half_max_count:
            # We've overflowed in the negative direction
            self.delta -= self.max_count + 1
        elif self.delta < self.neg_half_max_count:
            # We've overflowed in the positive direciton.
            self.delta += self.max_count + 1

        # Update the encoder count for the next iteration
        self.encoder_count = self.current_count

        # Update the position with the count change
        self.position += self.delta

    def get_position(self):
        """
        @brief Gets the most recent encoder position.
        @return The most recent encoder position.
        """
        return self.position

    def get_delta(self):
        """
        @brief Gets the most recent change in encoder count.
        @return The most recent change in encoder count (differential).
        """
       
        return self.delta

    def zero(self):
        """
        @brief Resets the encoder position to zero.
        @details This method resets the position to zero and accounts for value changes
        needed so that the next call to update() works as expected.
        """
        self.position = 0
        self.encoder_count = 0
        self.delta = 0
        self.current_count = 0

    def get_speed(self,encoderCPR,controlFrequency):
        
        self.encoderCPR = encoderCPR
        self.controlFrequency = controlFrequency
        self.speed = self.delta*(self.controlFrequency*60)/self.encoderCPR #RPM
        return self.speed 