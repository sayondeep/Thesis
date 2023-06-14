# Read hits.txt into a list of lists
hits_list = []
with open('hits_8.txt', 'r') as f:
    for line in f:
        # Split the line by whitespace and convert each value to float
        hits_list.append([float(value) for value in line.split()])

# Print the list of lists
print(hits_list)
# Calculate the average of each index across the list of lists
averages = [sum(column) / len(column) for column in zip(*hits_list)]

# Print the averages
print(averages)