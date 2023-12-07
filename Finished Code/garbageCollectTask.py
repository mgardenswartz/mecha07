# Import Python Garbage Collector
import gc

class garbageCollectTask:
    def run(self):
        """
        @brief Collects garbage one time.
        @details Clears heap and defragments.
        """
        while True:
            gc.collect()
            yield