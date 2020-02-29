def get_param_dict():

    db = open('database.txt', 'r')
    param = {}

    while True:

        line = db.readline()
        if not line:
            break

        key_and_val = line.split(':')
        key = key_and_val[0].strip()
        value = key_and_val[1].strip()

        if key == 'W' or key == 'H':
            param[key] = int(value)
        elif key == 'camera_offset' or key == 'tcp' or key == 'corridor' or key == 'kitchen' or key == 'dining':
            value = value.split(',')
            value = [float(item) for item in value]
            param[key] = tuple(value)
        else:
            param[key] = value

    db.close()

    return param


#############################################


if __name__ == '__main__':

    param = get_param_dict()
    print(param['tcp'])
