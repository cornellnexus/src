class DataBase: 
    def init(self):
        self.core_data = {"phase" : 0, 
                 "state" : [0, 0, 0], 
                 "time_step" : 0}

    def get_data(self): 
        return self.core_data 

    def update_data(self, name, value): 
        self.core_data[name] = value
