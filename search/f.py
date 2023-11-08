d = [1,2,3,4]
j = []
for i in range(10):
    try:
        print(d[i])
    except IndexError:
        print('wrong index')
