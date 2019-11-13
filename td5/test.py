import json
import numpy as np

f = open('linear_spline_example.json')

data = f.read()
datastore = json.loads(data)
posX = [None]*len(datastore['x'])
t = [None]*len(datastore['x'])
posY = [None]*len(datastore['x'])
posT = [None]*len(datastore['x'])
k=0
for d in datastore['x']:
    posX[k] = float(d[1])
    t[k] = float(d[0])
    k = k+1
k=0
for d in datastore['y']:
    posY[k] = float(d[1])
    k = k+1
k=0
for d in datastore['theta']:
    posT[k] = float(d[1])
    k = k+1

knots = [[[None], [None, None, None]] for _ in range(len(datastore['x']))]
for i in range(len(posX)):
    knots[i][0]= t[i]
    knots[i][1] = np.array([posX[i], posY[i], posT[i]])


print(knots[2][1][2])
