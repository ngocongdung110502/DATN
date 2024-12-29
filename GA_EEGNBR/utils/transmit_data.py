from time import process_time

from GA_EEGNBR.config.config_parser import parser

trans_parameter = parser['Transmit Data']

def transmit_data(path, source_id, packet_id):
    for i in range(len(path) - 1):
        transmitter = path[i]
        receiver = path[i+1]

        processing_time = int(trans_parameter['data_size']) / int(trans_parameter['bit_rate'])

        energy_consumed_transmitter = processing_time * float(trans_parameter['transmission_power'])
        energy_consumed_receiver = processing_time * float(trans_parameter['receiver_power'])

        transmitter.energy -= energy_consumed_transmitter
        receiver.energy -= energy_consumed_receiver

        # print(
        # f"{transmitter.node_id} (remaining energy: {transmitter.energy:.5f}J) -> {receiver.node_id} (remaining energy: {receiver.energy:.5f}J)")
        print(
            f"Packet {source_id}-{packet_id}: {transmitter.node_id} (remaining energy: {transmitter.energy:.5f}J) -> {receiver.node_id} (remaining energy: {receiver.energy:.5f}J)")

