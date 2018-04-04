import argparse
import logging
import socket
import ast

import slugs_logging
slugs_logger = logging.getLogger("slugs_logger")

from slugs_startup_server_standalone import SlugsSynthesisStandalone, SlugsExecutor

class UDP_Broadcastor(object):
    def __init__(self, UDP_IP="127.0.0.1", UDP_PORT=5005):
        self._UDP_IP = UDP_IP
        self._UDP_PORT = UDP_PORT
        self._broadcast_sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP

    def broadcast_msg(self, msg):
        self._broadcast_sock.sendto(msg, (self._UDP_IP, self._UDP_PORT))

class UDP_Listener(object):
    def __init__(self, UDP_IP="127.0.0.1", UDP_PORT=5010):
        self._listen_sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        #self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        #self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen_sock.bind((UDP_IP, UDP_PORT))

    def retrieve_msg(self):
        data, addr = self._listen_sock.recvfrom(1024) # buffer size is 1024 bytes
        return data

def convert_str_to_list(str_value, inputs, outputs):
    true_AP_idx_list = []
    for idx,x in enumerate(inputs+outputs):
        if int(str_value[idx]):
            true_AP_idx_list.append(x)
    return true_AP_idx_list

def convert_str_to_dict(str_value, inputs, outputs):
    true_AP_idx_dict = {}
    for idx,x in enumerate(inputs+outputs):
        true_AP_idx_dict.update({x: int(str_value[idx])})
    return true_AP_idx_dict

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="\
Slugs Executor. Expect msg of type str or dict.\n\
For string, please add quotations(') before sending.\n\
==== Sample Call ====\n\
python executor_standalone.py --ltl_filename  <slugsin_file> --option interactiveStrategy --wait_for_init_env 0\n\
=====================", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('ltl_filename', type=str, help='Specify .slugsin for now')
    parser.add_argument('--options', action='append', help='Synthesis options in SLUGS. Without --') # nargs='*'
    parser.add_argument('--wait_for_init_env', type=int, help='1 if wait for init env from user. 0 otherwise', default=0) # nargs='*'

    args, unknown = parser.parse_known_args()
    args.options = ['--'+x for x in args.options] if args.options is not None else []

    interactive_setup = False
    synthesis_action = SlugsSynthesisStandalone('SLUGS', args)

    slugs_logger.info('Started slugs action server. Waiting for request...')

    # Start listener
    udp_listener = UDP_Listener()

    # Start broadcaster
    udp_broadcast = UDP_Broadcastor()
    udp_broadcast_monitor = UDP_Broadcastor(UDP_PORT=5000)

    set_init_state = False
    incoming_msg = []
    while synthesis_action.get_execution_status():
        # only start if interactiveStrategy is called
        if "--interactiveStrategy" in synthesis_action._options and \
            synthesis_action._synthesis_done and synthesis_action._realizable and not interactive_setup:
            # initialize executor
            slugs_executor = SlugsExecutor(synthesis_action)
            slugs_logger.info('Running interactive strategy now ...')

            # start topics
            slugs_executor.set_inputs()
            slugs_executor.set_outputs()
            inputs = slugs_executor.get_inputs()
            outputs = slugs_executor.get_outputs()
            slugs_logger.info("Input list: {0}".format(inputs))
            slugs_logger.info("Output list: {0}".format(outputs))
            interactive_setup = True


        if interactive_setup:
            # select an init state automatically
            if not set_init_state and not args.wait_for_init_env:
                init_state_str =  slugs_executor.init_inputs_outputs_wrapper("")
                slugs_logger.info("Init state (true props):{0}".format(\
                    convert_str_to_list(init_state_str, inputs, outputs)))
                set_init_state = True

                # send msg
                broadcast_msg = str(convert_str_to_dict(init_state_str, inputs, outputs)) #"'"+init_state_str+"'"
                udp_broadcast.broadcast_msg(broadcast_msg)

            # get msg
            incoming_msg.append(udp_listener.retrieve_msg())

            if incoming_msg:
                current_input = incoming_msg.pop(0)
                # set init state
                if not set_init_state:
                    slugs_logger.debug("Getting init state")
                    set_init_state = True
                    if isinstance(ast.literal_eval(current_input), str):
                        current_output_str = slugs_executor.init_inputs_outputs_wrapper(\
                            ast.literal_eval(current_input))
                        broadcast_msg = str(convert_str_to_dict(init_state_str, inputs, outputs)) #"'"+current_output_str+"'"
                        slugs_logger.info("Init state (true props):{0}".format(\
                            convert_str_to_list(current_output_str, inputs, outputs)))

                    elif isinstance(ast.literal_eval(current_input), dict):
                        current_input_dict = ast.literal_eval(current_input)
                        current_output_key, current_output_value = \
                            slugs_executor.init_inputs_outputs_wrapper(\
                            current_input_dict.keys(), [bool(x) for x in current_input_dict.values()])
                        broadcast_msg = str(dict(zip(current_output_key,current_output_value)))

                    else:
                        slugs_logger.warning('Message format unknown:{0}.\nWe are skipping this msg.'.format(\
                            current_input))

                else:
                    slugs_logger.debug("Getting trans state")
                    if isinstance(ast.literal_eval(current_input), str):
                        current_output_str = slugs_executor.trans_inputs_wrapper(\
                            ast.literal_eval(current_input))
                        broadcast_msg = str(convert_str_to_dict(init_state_str, inputs, outputs)) #"'"+current_output_str+"'"
                        slugs_logger.info("Current state (true props):{0}".format(\
                            convert_str_to_list(current_output_str, inputs, outputs)))

                    elif isinstance(ast.literal_eval(current_input), dict):
                        current_input_dict = ast.literal_eval(current_input)
                        current_output_key, current_output_value = \
                            slugs_executor.trans_inputs_wrapper(\
                            current_input_dict.keys(), [bool(x) for x in current_input_dict.values()])
                        broadcast_msg = str(dict(zip(current_output_key,current_output_value)))

                    else:
                        slugs_logger.warning('Message format unknown:{0}.\nWe are skipping this msg.'.format(\
                            current_input))


            udp_broadcast.broadcast_msg(broadcast_msg)

            # format msg for monitor
            monitor_prop_dict = {}
            for idx, x in enumerate(current_output_key):
                if x in inputs:
                    monitor_prop_dict.update({"i:"+x: current_output_value[idx]})
                else:
                    monitor_prop_dict.update({"o:"+x: current_output_value[idx]})

            udp_broadcast_monitor.broadcast_msg(str(monitor_prop_dict))
            slugs_logger.info("Sent msg: {0}".format(broadcast_msg))

# ubuntu
# python executor_standalone.py --ltl_filename  /home/catherine/LTLMoP/src/examples/firefighting/firefighting.slugsin --option interactiveStrategy
