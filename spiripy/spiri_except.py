class SpiriGoConnectionError(Exception):
    def __init__(self, server = "unknown"):
        self.server = server
