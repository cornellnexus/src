# import logging
# import threading
# import time

# def retrieve_data(name):
#     logging.info("Thread %s: starting", name)
#     time.sleep(2)
#     logging.info("Thread %s: finishing", name)



# if __name__ == "__main__":
#     format = "%(asctime)s: %(message)s"
#     logging.basicConfig(format=format, level=logging.INFO,
#                         datefmt="%H:%M:%S")

#     logging.info("Main    : before creating thread")
#     packet_maker = threading.Thread(target=retrieve_data, args=(1,), daemon=True)
#     logging.info("Main    : before running thread")
#     packet_maker.start()
#     # m.execute_mission()

#     logging.info("Main    : wait for the thread to finish")
#     # x.join()
    
#     logging.info("Main    : all done")

