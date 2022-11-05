import time
from constants.definitions import CSV_PATH

def write_state_to_csv(predicted_state):
    with open(CSV_PATH + '/datastore.csv', 'a') as fd:
        fd.write(
            str(predicted_state[0])[1:-1] + ',' + str(predicted_state[1])[1:-1] + ',' + str(predicted_state[2])[1:-1] + '\n')
    time.sleep(0.001)

def write_phase_to_csv(phase):
    with open(CSV_PATH + '/phases.csv', 'a') as fd:
        fd.write(str(phase) + '\n')