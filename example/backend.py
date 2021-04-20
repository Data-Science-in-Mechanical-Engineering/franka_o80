import sys
import logging
import time
import o80
import signal_handler
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__)) + '/build')

SEGMENT_ID = "franka_o80_0"
FRANKA_IP = "192.168.0.1"
FREQUENCY = 500

def _run():

    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format=str("[ Franka o80 | segment_id: {} | bursting_mode: False |"+
                   " %(levelname)s %(asctime)s] %(message)s").format(SEGMENT_ID),
        level=logging.DEBUG,
        handlers=[log_handler]
    )

        
    logging.info("Clearning shared memory on {}".format(SEGMENT_ID))
    o80.clear_shared_memory(SEGMENT_ID)

    logging.info("Starting o80 standalone with frequency {}".format(FREQUENCY))
    franka_o80.start_standalone(SEGMENT_ID,
                                 FREQUENCY,
                                 False, # Bursting mode
                                 FRANKA_IP)

    signal_handler.init() # for detecting ctrl+c
    try:
        time_c = time.time()-6
        while not signal_handler.has_received_sigint():
            time.sleep(0.01)
            if time.time()-time_c > 5:
                logging.info("Running ...")
                time_c = time.time()
    except (KeyboardInterrupt,SystemExit):
        logging.info("Exiting ...")
    except Exception as e:
        logging.error(str(e))

    logging.info("Stopping franka o80 backend")
    franka_o80.stop_standalone(SEGMENT_ID)
    logging.info("Exiting")


if __name__ == "__main__":

    _run()
