import numpy as np
import math
from example import main
from ant_colony import AntColony
from obst import astar

def dmain():
    cities = []
    points = []
    hovtrans= []
    with open('./data/5.txt') as f:
        for line in f.readlines():
            city = line.split(' ')
            cities.append(dict(index=int(city[0]), x=int(city[1]), y=int(city[2]),z=int(city[3]),h=int(city[4])))
            points.append((int(city[1]), int(city[2]), int(city[3])))
            hovtrans.append(int(city[4]))
    print('pointss',points)
    x=[]
    y=[]
    z=[]
    for i in range(len(points)):
        k=points[i]
        x.append(k[0])
        y.append(k[1])
        z.append(k[2])
    mx=max(x)*4
    my=max(y)*4
    mz=max(z)*4
    xy = np.zeros((mx,my,mz))
    print(xy.shape)
    #xy[170,78,13]=1
    #print(xy[170,78,13])
    rr=[10,20,40,40,65]#x_axis starting_pt
    rc=[45,20,45,75,25]#y_axis starting_pt
    rd=[0,0,0,0,0]#z_axis starting_pt
    rx=[15,15,15,15,10]#span of X_axis as per rr
    ry=[15,10,15,15,15]#span of X_axis as per rr
    rz=[45,40,50,60,50]#span of X_axis as per rr
    jj=[]
    kk=[]
    for i in range(len(rr)):
        xy[rr[i]:rr[i]+rx[i]+1,rc[i]:rc[i]+ry[i]+1,rd[i]:rd[i]+rz[i]+1]=1
    maze=xy
    print(maze.shape)
    print(len(maze[0][0])-1)
    #maze=mazei[0:11,0:15,0:21]
    #print('kkj',maze[:,:,10])
    nm = np.full((len(x),len(y)),0.00)
    #print(maze[10][15])
    #print('jjkj',maze[0:10,0:10,0:5])
    #path=astar(maze, (0,0,0), (10,10,20))
    #print('fghfhgf',path)
    print(len(x))
    pati=[]
    for i in range(len(x)):
        for j in range(len(x)):
            start=points[i]
            end=points[j]
            path=astar(maze, start, end)
            print('patth',path)
            pati.append(path)
            xj=[]
            yj=[]
            zj=[]
            for m in range(len(path)):
                s=path[m]
                xj.append(s[0])
                yj.append(s[1])
                zj.append(s[2])
           # print('xj,yj',xj,yj)
            sum=0
            for n in range(len(xj)-1):  
                sum += math.sqrt((xj[n] - xj[n+1]) ** 2 + (yj[n] - yj[n+1]) ** 2 + (zj[n] - zj[n+1])**2)         
            nm[i,j]=sum
            print(nm)
    print('pati',len(pati))
    
    pati=np.array(pati)
    pati=np.reshape(pati, (len(points),len(points)))
    print('pati',pati)
    print(pati[1,2])
    print(pati.shape)
    np.fill_diagonal(nm, [np.inf])
    main(maze,nm,hovtrans,points,cities,rr,rc,rd,rx,ry,rz,pati)
    #ant_colony = AntColony(nm, 20, 1, 500, 0.95, alpha=1, beta=1)
    #shortest_path = ant_colony.run()
    #print ("shorted_path: {}".format(shortest_path))
if __name__ == '__main__':
    dmain()
