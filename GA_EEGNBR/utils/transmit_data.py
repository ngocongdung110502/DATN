from GA_EEGNBR.config.config_parser import parser
from GA_EEGNBR.src.sensor import distance


def transmit_data(path, source_id, packet_id):
    trans_parameter = parser['Transmit Data']

    for i in range(len(path) - 1):
        transmitter = path[i]
        receiver = path[i+1]

        distance_between_node = distance(transmitter, receiver)
        processing_time = int(trans_parameter['data_size']) / int(trans_parameter['bit_rate'])
        transmission_time = distance_between_node / int(trans_parameter['speed_in_water'])
        total_transmission_time = processing_time + transmission_time

        energy_consumed_transmitter = total_transmission_time * float(trans_parameter['transmission_power'])
        energy_consumed_receiver = processing_time * float(trans_parameter['receiver_power'])

        transmitter.energy -= energy_consumed_transmitter
        receiver.energy -= energy_consumed_receiver

        # print(
        # f"{transmitter.node_id} (remaining energy: {transmitter.energy:.5f}J) -> {receiver.node_id} (remaining energy: {receiver.energy:.5f}J)")
        print(
            f"Packet {source_id}-{packet_id}: {transmitter.node_id} (remaining energy: {transmitter.energy:.5f}J) -> {receiver.node_id} (remaining energy: {receiver.energy:.5f}J)")

