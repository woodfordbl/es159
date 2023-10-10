import numpy as np
import csv

# Initialize the error list
errors = []

for i in range(0, 6):

    error_list = []
    # Open csv file
    with open('/Users/woodfordb/Downloads/es159_lab0/lab1/files/measured_and_predicted_values.csv', mode='r') as file:
        
        # Define the CSV reader
        csv_reader = csv.reader(file)

        # Skip the header row
        next(csv_reader)
    
        for row in csv_reader:

            if row[0] == str(i):
                # Convert the string to a numpy array
                predicted = np.fromstring(row[1][1:-1], sep=' ')
                actual = np.fromstring(row[2][1:-1], sep=' ')

                # Calculate the mean error and std dev for each position
                error = np.subtract(predicted, actual)
                error_list.append(list(error))
        
    errors.append(error_list)

np_errors = np.array(errors)

# Calculate the mean error and std dev for each column in each position
for i, matrix in enumerate(np_errors):
    means = np.mean(matrix, axis=0)
    std_devs = np.std(matrix, axis=0)
    print()
    print(f"Mean for position {i}: {means}")
    print()
    print(f"Std Dev for position {i}: {std_devs}")
    print()
    print("––"*20)