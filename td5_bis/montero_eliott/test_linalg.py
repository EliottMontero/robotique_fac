import numpy as np
import json

path = "cubic_spline_example.json"
#On ouvre le json
f = open(path)
data = f.read()
datastore = json.loads(data)
#initialisation des listes ou on va mettre les donnees
posX = [None]*len(datastore['x'])
dvX = [None]*len(datastore['x'])
t = [None]*len(datastore['x'])
posY = [None]*len(datastore['y'])
dvY = [None]*len(datastore['y'])
posT = [None]*len(datastore['theta'])
dvT = [None]*len(datastore['theta'])
k=0
#on recupere les donnees
for d in datastore['x']:
    dvX[k] = float(d[2])
    posX[k] = float(d[1])
    t[k] = float(d[0])
    k = k+1
k=0
for d in datastore['y']:
    dvY[k] = float(d[2])
    posY[k] = float(d[1])
    k = k+1
k=0
for d in datastore['theta']:
    dvT[k] = float(d[2])
    posT[k] = float(d[1])
    k = k+1

#on initialise les listes pour y mettre les points de controles par dimension
knotsX = [[[None], [None], [None]] for _ in range(len(datastore['x']))]
knotsY = [[[None], [None], [None]] for _ in range(len(datastore['y']))]
knotsT = [[[None], [None], [None]] for _ in range(len(datastore['theta']))]
for i in range(len(posX)):
    knotsX[i][0] = t[i]
    knotsX[i][1] = posX[i]
    knotsX[i][2] = dvX[i]
for i in range(len(posY)):
    knotsY[i][0] = t[i]
    knotsY[i][1] = posY[i]
    knotsY[i][2] = dvY[i]
for i in range(len(posT)):
    knotsT[i][0] = t[i]
    knotsT[i][1] = posT[i]
    knotsT[i][2] = dvT[i]

knots = knotsX
print(knots)

for i in range(len(knots)-1):
    x1 = knots[i][0]
    x2 = knots[i+1][0]
    y1 = knots[i][1]
    y1_2 = knots[i+1][1]
    y2 = knots[i][2]
    y2_2 = knots[i+1][2]

    eqX = np.array([[x1**3, x1**2, x1, 1], [x2**3, x2**2, x2, 1],\
     [3*(x1**2), 2*x1, 1, 0], [3*(x2**2), 2*x2, 1, 0]])
    eqY = np.array([y1, y1_2, y2, y2_2])

    res = np.linalg.solve(eqX, eqY)
    a = res[0]
    b = res[1]
    c = res[2]
    d = res[3]

    pos = a*(t**3) + b*(t**2) +c*(t) +d)
    speed = 3*a*(t**2)+2*b*t + c
    print(res)
