def calculate_utility(props):
    # If configuration uses too much power, discourage by setting utility to negative power draw
    if not props[1]:
        return -1 * props[0]
    # Otherwise, if the power usage is fine, 
    return props[2]