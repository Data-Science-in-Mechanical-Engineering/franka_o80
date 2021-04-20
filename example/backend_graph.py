import sys
import logging
import time
import o80
import signal_handler
from functools import partial
from fyplot import function_plot
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__)) + '/build')

SEGMENT_ID = "franka_o80_0"
FRANKA_IP = "192.168.0.1"
FREQUENCY = 500
WINDOW = (1200,800)


def _get_plot(frontend):

    plt = function_plot.Plot(SEGMENT_ID,0.1,WINDOW)

    class Iter:
        iteration = 0
    
    def get_joint(dof):
        latest = frontend.latest().get_iteration()
        if latest>Iter.iteration:
            obs = frontend.read(Iter.iteration)
            if dof==1:
                Iter.iteration+=1
            return obs.get_desired_states().get(dof).get()
        return None
    
    desired_states_plot = ( (partial(get_joint,0),(0,255,0)),
                            (partial(get_joint,1),(255,0,0)) )
    
    plt.add_subplot(FRANKA_IP,1000,desired_states_plot)
    
    return plt


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

    logging.info("Instantiating frontend")
    frontend = franka_o80.FrontEnd(SEGMENT_ID)

    logging.info("Starting plotting")
    plot = _get_plot(frontend)
    plot.start()

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

    logging.info("Stopping plot")
    plot.stop()
    logging.info("Stopping franka o80 backend")
    franka_o80.stop_standalone(SEGMENT_ID)
    logging.info("Exiting")


if __name__ == "__main__":

    _run()
