import numpy as np
from packet import GUIDEPacket, QueryPacket, AckPacket, DataPacket

NINF = float('-inf')

class SensorNode:
    def __init__(self, id, position, is_sink=False,is_source=False, bit_rate=10e3, speed_of_sound=1500, initial_energy=30, tx_power=2, rx_powver=0.1):
        self.id = id
        self.position = position
        self.gpsn = 0
        self.hc = NINF
        self.neighbors = []
        self.is_sink = is_sink
        self.is_source = is_source
        self.memory_queue = set()
        self.energy = initial_energy
        self.bit_rate = bit_rate
        self.speed_of_sound = speed_of_sound
        self.tx_power = tx_power
        self.rx_power = rx_powver
    
    # function define delay for transmisson and reception
    def transmission_delay(self, packet_size):
        return packet_size/self.bit_rate
    
    # function define delay for propagation
    def propagation_delay(self, distance):
        return distance/self.speed_of_sound
    
    # function define consume energy of every node
    def consume_energy(self, mode, packet_size):
        if mode == 'tx':
            self.energy -= self.tx_power * self.transmission_delay(packet_size)
        elif mode == 'rx':
            self.energy -= self.rx_power * self.transmission_delay(packet_size)

    # function define update guiding network
    def update_with_packet(self, packet, packet_size):
        self.consume_energy('rx', packet_size) #consume energy for reception

        if packet.gpsn > self.gpsn:
            self.gpsn = packet.gpsn
            self.hc = packet.hc + 1
            return True
        elif packet.gpsn == self.gpsn and self.hc > packet.hc + 1:
            self.hc = packet.hc + 1
            return True
        return False
    
    # broadcast guide packet for neighbor nodes
    def broadcast_packet(self, packet, packet_size):
        updated_nodes = []

        self.consume_energy('tx', packet_size)

        for neighbor in self.neighbors:
            if neighbor.update_with_packet(packet, packet_size):
                new_packet = GUIDEPacket(packet.gpsn, neighbor.hc)
                updated_nodes.append((neighbor, new_packet))
        return updated_nodes
    
    # function defines send query packet to node
    def send_query_packet(self, query_packet, communication_radius, packet_size): 
        ack_packets = []

        self.consume_energy('tx', packet_size)

        for neighbor in self.neighbors:
            if np.linalg.norm(self.position - neighbor.position) <= communication_radius:
                ack_packet = neighbor.send_ack_packet(query_packet, communication_radius, packet_size)
                if ack_packet:
                    ack_packets.append(ack_packet)

        #timer.expire()

        return ack_packets
    
    def send_ack_packet(self, query_packet, communication_radius, packet_size):
        self.consume_energy('tx', packet_size)

        if query_packet.hc > self.hc and query_packet.sender_id not in self.memory_queue:
            ack_packet = AckPacket(receiver_id=query_packet.sender_id, sender_id=self.id, energy=self.energy, hc=self.hc)
            self.memory_queue.add(query_packet.sender_id)
            timer = Timer(2 * communication_radius / self.speed_of_sound)

            new_query_packet = QueryPacket(self.id, query_packet.source_id, query_packet.dpsn, self.hc)
            self.send_query_packet(new_query_packet, communication_radius, packet_size)
            return ack_packet
        return None
    
    def select_optimal_node(self, ack_packets, energy_threshold):
        optimal_node = None
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
            
            optimal_node = max(candidates, key=compute_priority, default=None)

        return optimal_node
    
    def generate_and_send_data(self, packet_id, communication_radius, energy_threshold):
        current_node = self
        delay = 0
        data_packet = DataPacket(packet_id, self.id, None, 50)
        while not current_node.is_sink:
            ack_packets = []
            for neighbor in current_node.neighbors:
                query_packet = QueryPacket(current_node.id, current_node.id, data_packet.packet_id, current_node.hc)
                timer = Timer(2 * communication_radius / current_node.speed_of_sound)
                ack_packet = neighbor.receive_query_packet(query_packet, communication_radius)
                if ack_packet:
                    ack_packets.append(ack_packet)
            optimal_relay = current_node.select_optimal_relay(ack_packets, energy_threshold)
            if optimal_relay:
                next_node = next(n for n in current_node.neighbors if n.id == optimal_relay.sender_id)
                delay += current_node.transmission_delay(data_packet.size) + current_node.propagation_delay(np.linalg.norm(current_node.position - next_node.position))
                current_node.consume_energy('tx', data_packet.size)
                next_node.consume_energy('rx', data_packet.size)
                current_node = next_node
            else:
                print(f"No optimal relay found from node {current_node.id}")
                break
        return delay
    
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