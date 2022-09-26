from csv import reader 
import csv

with open('gps_4-25.csv', 'r') as read_obj:
    csv_reader = reader(read_obj)
    return_array = []
    for read in csv_reader: 
        # row = read[0]
        # row = row.replace('[', '')
        # row = row.replace(']', '')

        # current_index = row.index("(")
        # row = row[:current_index+1] + "'x': " + row[current_index+1:]
        # current_index2 = row.index(',', current_index)
        # row = row[:current_index2+1] + " 'y':" + row[current_index2+1:]
        # current_index3 = row.index(',', current_index2+1)
        # row = row[:current_index3+1] + " 'z':" + row[current_index3+1:]

        # current_index4 = row.index("(", current_index3)
        # row = row[:current_index4+1] + "'x': " + row[current_index4+1:]
        # current_index5 = row.index(',', current_index4)
        # row = row[:current_index5+1] + " 'y':" + row[current_index5+1:]
        # current_index6 = row.index(',', current_index5+1)
        # row = row[:current_index6+1] + " 'z':" + row[current_index6+1:]

        # current_index7 = row.index("(", current_index6)
        # row = row[:current_index7+1] + "'x': " + row[current_index7+1:]
        # current_index8 = row.index(',', current_index7)
        # row = row[:current_index8+1] + " 'y':" + row[current_index8+1:]
        # current_index9 = row.index(',', current_index8+1)
        # row = row[:current_index9+1] + " 'z':" + row[current_index9+1:]

        # row = row.replace("(","{")
        # row = row.replace(")","}")
        # row +=  "\n"
        row = read[0]
        # row = row.replace('"lon"','lon')
        # row = row.replace('"lat"', 'lat')
        # row = row[1:len(row)-2]
        row +=  "\n"
        return_array.append(row)
    
    with open("data2.csv", "w") as f:
        for row in return_array:
            f.write(row)
    f.close()

        

        
        
        