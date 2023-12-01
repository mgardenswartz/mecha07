def csvread(filename):
    # Create an empty list.
    filedata = []

    # Read the data as fast as possible.
    with open(filename, 'r') as csvfile:
        for line in csvfile:
            filedata.append(line)

    # Grab the title
    titles = filedata[0]
    # Split into a list of two titles.
    titles = titles.split(',')
    # Strip the titles.
    for idx,element in enumerate(titles):
        titles[idx] = element.strip()   
    # Delete more than two columns of titles.
    del titles[2:]
    # Delete the titles from the filedata
    filedata = filedata[1:]

    # Remove all whitespace
    for idx,element in enumerate(filedata):
        filedata[idx] = element.strip()
    
    # Split into a list of lists.   
    for idx,element in enumerate(filedata):
        filedata[idx] = filedata[idx].split(',')

    # Remove third or more columns.
    for row_index,row in enumerate(filedata):
        filedata[row_index] = row[:2] 

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
                    elif (char.isdigit() == True or char=="-" or char=="."):
                        temp += char
                except AttributeError: 
                    filedata[row_index] == [""]
            # Rewrite the comment-removed data back to filedata.
            filedata[row_index][column_index] = temp

    # Remove non-numbers.
    for row_index,row in enumerate(filedata):
        for column_index,value in enumerate(element):
            try:
                float(value)
            except ValueError:
                # Remove that line from the filedata
                filedata[row_index] == [""]

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

    # Sort into columns
    x_values =[]
    y_values =[]
    for row in filedata:
        x_values.append(float(row[0]))
        y_values.append(float(row[1]))

    return x_values,y_values,titles
