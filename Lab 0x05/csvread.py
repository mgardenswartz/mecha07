def csvread(filename):
    # Create an empty list.
    filedata = []

    # Read the data as fast as possible.
    with open(filename, 'r') as csvfile:
        for line in csvfile:
            filedata.append(line)

    # Remove all whitespace
    for idx,element in enumerate(filedata):
        filedata[idx] = element.strip()
    
    # Split into a list of lists.   
    for idx,element in enumerate(filedata):
        filedata[idx] = filedata[idx].split(',')

    # Remove comments
    for row_index,row in enumerate(filedata):
        # Remove comments
        for column_index,value in enumerate(row):
            temp = ""
            for char in value:
                try:
                    if char == "#":
                        break
                    # Allowable characters
                    else: #(char.isdigit() == True or char=="-" or char=="."):
                        temp += char
                except AttributeError: 
                    filedata[row_index] == [""]
            # Rewrite the comment-removed data back to filedata.
            filedata[row_index][column_index] = temp

    # Remove blank rows
    for idx,element in enumerate(filedata):
        for item in element:
            if item == '':
                del filedata[idx]   

    # Remove rows that have a blank entry
    for idx,element in enumerate(filedata):
        for cell,item in enumerate(element):
            if item == '':
                del filedata[idx]   
            
    # Remove blank rows.
    for row_index,row in enumerate(filedata):
        if row == [""]:
            del filedata[row_index]
    
    return filedata

# filedata = csvread('IMU_cal_coeffs.txt')

# first_row = filedata[0]
# cal_coeffs = [(int(hex_val,16) & 0x7F) - (int(hex_val,16) & 0x80) for hex_val in first_row]

# print(cal_coeffs)

