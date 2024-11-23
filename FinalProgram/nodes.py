import numpy as np
from packet import GuidePacket, AckPacket

NINF = float('-inf')

class SensorNode:
    def __init__(self, id, position, is_sink=False, is_source=False, bit_rate=10e3, speed_of_sound=1500,
                 initial_energy=30, tx_power=2, rx_powver=0.1):
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
        return packet_size / self.bit_rate

    # function define delay for propagation
    def propagation_delay(self, distance):
        return distance / self.speed_of_sound

    def consume_energy_tx(self, packet_size, distance):
        self.energy -= self.tx_power * (self.transmission_delay(packet_size) + distance / self.speed_of_sound)

    def consume_energy_rx(self, packet_size):
        self.energy -= self.rx_power * (self.transmission_delay(packet_size))

    # function define update guiding network
    def update_with_packet(self, packet, packet_size):
        self.consume_energy_rx(packet_size)  # consume energy for reception

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

        for neighbor in self.neighbors:
            if neighbor.update_with_packet(packet, packet_size):
                distance = np.linalg.norm(self.position - neighbor.position)
                self.consume_energy_tx(packet_size, distance)

                new_packet = GuidePacket(packet.gpsn, neighbor.hc)
                updated_nodes.append((neighbor, new_packet))
        return updated_nodes

    # function defines send query packet to node
    def send_query_packet(self, query_packet, communication_radius, packet_size):
        ack_packets = []

        for neighbor in self.neighbors:
            distance = np.linalg.norm(self.position - neighbor.position)
            if distance <= communication_radius:
                self.consume_energy_tx(packet_size, distance)  # node hiện tại truyền gói tin query

                neighbor.consume_energy_rx(packet_size)  # node hàng xóm nhận gói query
                ack_packet = neighbor.send_ack_packet(query_packet, communication_radius, packet_size)
                neighbor.consume_energy_tx(packet_size, distance)  # node hàng xóm gửi gói tin ack
                if ack_packet:
                    self.consume_energy_rx(packet_size)  # node hiện tại nhận gói tin query
                    ack_packets.append(ack_packet)

        return ack_packets

    def send_ack_packet(self, query_packet, communication_radius, packet_size):
        if query_packet.hc > self.hc and query_packet.sender_id not in self.memory_queue:
            ack_packet = AckPacket(receiver_id=query_packet.sender_id, sender_id=self.id, energy=self.energy,
                                   hc=self.hc)
            self.memory_queue.add(query_packet.sender_id)

            return ack_packet
        return None

    def select_optimal_node(self, ack_packets, energy_threshold):
        optimal_node = None
        min_hc = min((ack_packet.hc for ack_packet in ack_packets if ack_packet), default=NINF)
        candidates = [ack_packet for ack_packet in ack_packets if ack_packet and ack_packet.hc == min_hc]
        max_priority = NINF

        if candidates:
            for packet in candidates:
                try:
                    next_node = next(n for n in self.neighbors if n.id == packet.sender_id)
                    distance = np.linalg.norm(self.position - next_node.position)

                    if next_node.energy < energy_threshold:
                        print(f"Node {next_node.id} does not have enough energy. Skipping...")
                        continue
                    else:
                        energy_component = next_node.energy - energy_threshold

                    priority = energy_component / distance
                    if priority > max_priority:
                        max_priority = priority
                        optimal_node = packet
                except StopIteration:
                    continue
        if max_priority == NINF:
            # print("Không tìm thấy node tối ưu vì tất cả đều có độ ưu tiên = -NINF.")
            return None

        return optimal_node

    def send_data_packet(self, data_packet, next_node, distance):
        self.consume_energy_tx(data_packet.size, distance)
        next_node.receive_data_packet(data_packet)

    def receive_data_packet(self, data_packet):
        self.consume_energy_rx(data_packet.size)

