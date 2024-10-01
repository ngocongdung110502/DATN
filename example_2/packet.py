class GUIDEPacket:
    def __init__(self, gpsn, hc):
        self.gpsn = gpsn
        self.hc = hc

class QueryPacket:
    def __init__(self, sender_id, source_id, dpsn, hc):
        self.sender_id = sender_id
        self.source_id = source_id
        self.dpsn = dpsn
        self.hc = hc

class AckPacket:
    def __init__(self, receiver_id, sender_id, energy, hc):
        self.receiver_id = receiver_id
        self.sender_id = sender_id
        self.energy = energy
        self.hc = hc

class DataPacket:
    def __init__(self, update_state, source_id, receiver_id, dpsn, data_size):
        self.update_state = update_state
        self.source_id = source_id
        self.receiver_id = receiver_id
        self.dpsn = dpsn
        self.data_size = data_size
    
class CheckPacket:
    def __init__(self, update_state):
        self.update_state = update_state
