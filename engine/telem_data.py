class TelemData: 
  def __init__(self): 
    self.gps_current_pos_x = None 
    self.gps_current_pos_y = None 
    self.gps_last_pos_x = None 
    self.gps_last_pos_y = None 
    self.kalman_pos_x = None 
    self.kalman_pos_y = None 
    self.state = None 

  def get_data(self): 
    telem_data = {
      "gps_current_pos_x": self.gps_current_pos_x,
      "gps_current_pos_y": self.gps_current_pos_y, 
      "gps_last_pos_x": self.gps_last_pos_x, 
      "gps_last_pos_y": self.gps_last_pos_y,
      "kalman_pos_x": self.kalman_pos_x, 
      "kalman_pos_y": self.kalman_pos_y 
    }
    return telem_data

    
    



