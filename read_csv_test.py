import csv

def read_all_blocks(csv_file='path.csv'):
    """Read all individual blocks/cells from CSV and return them as a sequence with height 4"""
    blocks = []
    try:
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if len(row) >= 2:
                    x, y = float(row[0]), float(row[1])
                    blocks.append([x, y])
    except FileNotFoundError:
        print(f"CSV file {csv_file} not found")
        return []
    
    if not blocks:
        return []
    
    # Normalize blocks similar to waypoints but with height 4
    global offset_x, offset_y
    offset_x, offset_y = blocks[0][0], blocks[0][1]
    normalized_blocks = []
    for block in blocks:
        # Convert to tuples with height 4
        normalized_blocks.append(((block[0] - offset_x)/10, -(block[1] - offset_y)/10, 0.23))
    
    return normalized_blocks

print(read_all_blocks())