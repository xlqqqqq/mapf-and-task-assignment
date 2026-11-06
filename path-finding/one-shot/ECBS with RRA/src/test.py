obstacles = [
    (x + i, y + j)
    for x in (4, 11, 18)
    for y in (2, 6, 10)
    for i in range(6)
    for j in range(2)
]

for ele in obstacles:
    print(ele)
