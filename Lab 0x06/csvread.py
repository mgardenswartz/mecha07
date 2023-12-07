"""!
@file csvread.py
This file contains a function to read a text/CSV file
and return the data contained.

@author Max Gardenswartz
@date   2023-Oct-01 MLG Approximate date of creation of file
@date   2023-Dec-06 MLG Fixed an error with ==.

It is intended for educational use only, but its use is not limited thereto.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

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
                    filedata[row_index] = [""]
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