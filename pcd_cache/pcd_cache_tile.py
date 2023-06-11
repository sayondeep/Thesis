import csv

cache_hits=0
cache_miss=0

def calculate_data_value(data,curr_time):
    # p = 1  # Coefficient or factor (can be adjusted based on your requirements)
    p = data['p']
    # life = data['life']  # Lifetime of the data
    life = data['valid_till']- curr_time
    size = data['size']  # Size of the data
    
    value = p * life * (size/(6.25 * 10**6))
    return value


class RSU:
    def __init__(self, remaining_cache_size):
        self.remaining_cache_size = remaining_cache_size
        self.cached_data = {}

    def cache(self, data):
        self.cached_data[data['id']] = data
        self.remaining_cache_size -= data['size']
        # print(f"Cached data: {data['id']}")
    
    # def remove(self, data_id):
    #     del self.cached_data[data_id]

    def get_least_value_data(self):
        ele = min(self.cached_data.values(), key=lambda x: x['value'])
        # print(type(ele))
        # print(ele)
        return ele

    def total_cached_value(self):
        return sum(data['value'] for data in self.cached_data.values())

    def process_data(self, data):
        if self.remaining_cache_size >= data['size']:
            self.cache(data)
            print(f"{data['id']} Newly cached")
        else:
            for data_id in self.cached_data:
                self.cached_data[data_id]['value'] = calculate_data_value(self.cached_data[data_id],data['time'])
            
            Sleast_value = []
            while self.remaining_cache_size < data['size']:
                Dmin = self.get_least_value_data()
                # print(type(Dmin))
                # print(Dmin)
                Sleast_value.append(Dmin)
                # self.remove(Dmin['id'])
                print("Dropping due to no space calculation.")
                self.drop(Dmin)
                # print(Dmin)

            # print(Sleast_value)
            # if data['value'] > sum(d['value'] for d in Sleast_value):
            if calculate_data_value(data,data['time']) > sum(d['value'] for d in Sleast_value):
                self.cache(data)
                print(f"{data['id']} Newly Cached.")
            else:
                # self.drop(data)
                print(f"{data['id']} Caching Rejected.")
                for d in Sleast_value:
                    self.cache(d)
                    print(f"{d['id']} ReCached.")


    def drop(self, data):
        # Function to handle dropping of data
        data_id = data['id']
        if data_id in self.cached_data:
            del self.cached_data[data_id]
            self.remaining_cache_size += data['size']
            print(f"Dropped data: {data_id}")
            



# Function to read data from CSV file
def read_data_from_csv(Ri,filename):
    global cache_hits, cache_miss  # Declare variables as global
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            data = {
                'id': row['Tile Name'],
                # 'life': float(row['Valid_Till']) - int(row['Time']) if row['Valid_Till'] != 'inf' else 100,
                'time': int(row['Time']),
                'size': float(row['Size']),
                'p': float(row['Probability']),
                'valid_till': float(row['Valid_Till'])

            }
            if data['id'] not in Ri.cached_data or int(row['Time']) >= float(Ri.cached_data[data['id']]['valid_till']):
                cache_miss+=1
                Ri.process_data(data)
                
            else:
                cache_hits+=1

# Example usage
# Ri = RSU(400)  # Initialize RSU with remaining cache size of 100

# # Example data
# data1 = {'id': 'data1', 'life': 5, 'size': 20}
# data2 = {'id': 'data2', 'life': 7, 'size': 30}
# data3 = {'id': 'data3', 'life': 4, 'size': 15}
# data4 = {'id': 'data4', 'life': 6, 'size': 10}

# Ri.process_data(data1)
# print('\n')
# Ri.process_data(data2)
# print('\n')
# Ri.process_data(data3)
# print('\n')
# Ri.process_data(data4)
# print('\n')

# # Check the cached data
# print("Cached data:")
# for data in Ri.cached_data.values():
#     print(data)


# Read data from CSV file
# filename = 'validity_with_size.csv'  # Replace with the actual filename/path
# read_data_from_csv(filename)

# # Check the cached data
# print("Cached data:")
# # for data in Ri.cached_data.values():
# #     print(data)
# print("Cache_hits: ",cache_hits)
# print("Cache_miss: ",cache_miss)


def get_readings(cache_size):
    global cache_hits,cache_miss
    cache_hits=0
    cache_miss=0
    Ri = RSU(cache_size)
    filename = 'validity_with_size.csv'  # Replace with the actual filename/path
    read_data_from_csv(Ri,filename)
    print("Cache_hits: ",cache_hits)
    print("Cache_miss: ",cache_miss)

    return cache_hits,cache_miss

hits=[]
miss=[]
for i in range(70, 201, 10):
    h,m = get_readings(i)
    hits.append(h)
    miss.append(m)

print(hits)
print(miss)