# The MovingAverage implements the concept of window in a set of measurements.
# The window_size is the number of most recent measurements used in average.
# The measurements are continuously added using the method add.
# The get_average method must return the average of the last window_size measurements.
from collections import deque

class MovingAverage:
    value_queue = deque()
    window_size = 1
    
    def __init__(self, window_size):
        self.window_size = window_size
        pass

    # add a new measurement
    def add(self, val):
        if ( len(self.value_queue) < self.window_size ): 
            self.value_queue.append(val)
        else: 
            self.value_queue.popleft()
            self.value_queue.append(val)
     
    # return the average of the last window_size measurements added 
    # or the average of all measurements if less than window_size were provided
    # if no values have been added, return 0
    def get_average(self):
        return sum(self.value_queue) / (max(1, len(self.value_queue)))