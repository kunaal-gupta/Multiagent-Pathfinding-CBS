for i in range(10):
    for j in range(5):
        print(i, j)
        if j == 3:
            print('breaking')
            break
    break
