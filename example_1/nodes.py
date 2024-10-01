import numpy as np

NINF = float('-inf')

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

class SensorNode:
    def __init__(self, id, position, is_sink=False):
        self.id = id
        self.position = position
        self.gpsn = 0
        self.hc = NINF
        self.neighbors = []
        self.is_sink = is_sink
        self.memory_queue = set()
        self.energy = np.random.uniform(0.5, 1.0)

    def update_with_packet(self, packet):
        if packet.gpsn > self.gpsn:
            self.gpsn = packet.gpsn
            self.hc = packet.hc + 1
            return True
        elif packet.gpsn == self.gpsn and self.hc > packet.hc + 1:
            self.hc = packet.hc + 1
            return True
        return False

    def broadcast_packet(self, packet):
        updated_nodes = []
        for neighbor in self.neighbors:
            if neighbor.update_with_packet(packet):
                new_packet = GUIDEPacket(packet.gpsn, neighbor.hc)
                updated_nodes.append((neighbor, new_packet))
        return updated_nodes

    def send_query_packet(self, query_packet, communication_radius, timer):
        ack_packets = []
        for neighbor in self.neighbors:
            if np.linalg.norm(self.position - neighbor.position) <= communication_radius:
                ack_packet = neighbor.receive_query_packet(query_packet, communication_radius)
                if ack_packet:
                    ack_packets.append(ack_packet)
        timer.expire()
        return ack_packets

    def receive_query_packet(self, query_packet, communication_radius):
        if query_packet.hc > self.hc and query_packet.sender_id not in self.memory_queue:
            ack_packet = AckPacket(receiver_id=query_packet.sender_id, sender_id=self.id, energy=self.energy, hc=self.hc)
            self.memory_queue.add(query_packet.sender_id)
            timer = Timer(2 * communication_radius / 1500)
            self.send_query_packet(query_packet, communication_radius, timer)
            return ack_packet
        return None

    def select_optimal_relay(self, ack_packets, energy_threshold):
        optimal_relay = None
        min_hc = min((ack_packet.hc for ack_packet in ack_packets if ack_packet), default=NINF)
        candidates = [ack_packet for ack_packet in ack_packets if ack_packet and ack_packet.hc == min_hc]

        if candidates:
            def compute_priority(packet):
                try:
                    sender_node = next(n for n in self.neighbors if n.id == packet.sender_id)
                    distance = np.linalg.norm(self.position - sender_node.position)
                    energy_component = max(0, packet.energy - energy_threshold)
                    return energy_component / distance
                except StopIteration:
                    return NINF
            
            optimal_relay = max(candidates, key=compute_priority, default=None)

        return optimal_relay

class Timer:
    def __init__(self, duration):
        self.duration = duration
        self.remaining_time = duration

    def tick(self, time_passed):
        self.remaining_time -= time_passed

    def is_expired(self):
        return self.remaining_time <= 0

    def expire(self):
        self.remaining_time = 0
