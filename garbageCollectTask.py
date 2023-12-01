import gc

class garbageCollectTask:
    def run(self):
        while True:
            gc.collect()
            yield
            