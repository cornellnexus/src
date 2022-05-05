import statistics

def get_median(data_list):
    '''

    Args:
        data_list: list of floats

    Returns: a single string of the median of [data_list]

    '''
    return str(statistics.median(data_list))

def get_medians(data):
    '''

    Args:
        data: list of list of floats
        num_inputs: number of inputs


    Returns: a tuple of strings where the first and second entries are the medians of [coords]

    '''
    parameters = [] # [[],[],[]]
    parameter_medians = []

    num_inputs = len(data[0])
    for i in range(num_inputs):
        parameters.append([])

    for packet in data: #[8.01, 0.01, 0.0]
        for i in range(num_inputs):
            parameters[i].append(packet[i])

    print("PARAMETERS", parameters)
    for i in range(num_inputs):
        parameter_medians.append(get_median(parameters[i]))

    print(parameter_medians)
    return parameter_medians


get_medians([[8.01, 0.01, 0.0], [8.01, 0.01, 0.0], [8.01, 0.01, 0.0], [8.01, 0.01, 0.0], [8.01, 0.01, 0.0]])