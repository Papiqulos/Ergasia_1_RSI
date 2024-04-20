import numpy as np

def find_key_by_value(tree, target_value):
    for key, value in tree.items():
        for v in value:
            if np.all(target_value == v):
                return key
    return None



d = {1: [(2, 3), (4, 5)], 2: [(3, 4), (5, 6)]}

print(find_key_by_value(d, (3, 4))) 
