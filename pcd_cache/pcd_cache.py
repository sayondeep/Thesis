def calculate_data_value(data):
    p = 1  # Coefficient or factor (can be adjusted based on your requirements)
    life = data['life']  # Lifetime of the data
    size = data['size']  # Size of the data
    
    value = p * life * size
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
                self.cached_data[data_id]['value'] = calculate_data_value(self.cached_data[data_id])
            
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
            if calculate_data_value(data) > sum(d['value'] for d in Sleast_value):
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
            


# Example usage
Ri = RSU(50)  # Initialize RSU with remaining cache size of 100

# Example data
data1 = {'id': 'data1', 'life': 5, 'size': 20}
data2 = {'id': 'data2', 'life': 7, 'size': 30}
data3 = {'id': 'data3', 'life': 4, 'size': 15}
data4 = {'id': 'data4', 'life': 6, 'size': 10}

Ri.process_data(data1)
print('\n')
Ri.process_data(data2)
print('\n')
Ri.process_data(data3)
print('\n')
Ri.process_data(data4)
print('\n')

# Check the cached data
print("Cached data:")
for data in Ri.cached_data.values():
    print(data)
