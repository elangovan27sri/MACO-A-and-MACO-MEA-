import numpy as np
import math
from example import main
from ant_colony import AntColony
import glob
def dmain():
    cities = []
    points = []
    hovtrans= []
    with open('./meastr_data/5.txt') as f:
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
    rr=[10,20,40,40,65]
    rc=[45,20,45,75,25]
    rd=[0,0,0,0,0]
    rx=[15,15,15,15,10]
    ry=[15,10,15,15,15]
    rz=[45,40,50,60,50]
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
    print('nm',nm)
    print(len(x))
    pati=np.full((len(x),len(y)),list)
    print('kjhkjh',pati)
    for txt in glob.glob("./meastr_data/5obstpaths/*.txt"):
        print(txt)
        with open(txt) as f:
            h=str(f.name)
            i=int(h[-6])
            j=int(h[-5])
            xj=[]
            yj=[]
            zj=[]
            pa=[]
            for line in f.readlines():
                city = line.split('    ')
                yj.append(int(city[0]))
                xj.append(int(city[1]))
                zj.append(15)
                pa.append((int(city[1]),int(city[0]),15))
            print('pa',pa)
            pati[i,j]=pa
            
            paa = pa[::-1]
            print('reversed',paa)
            pati[j,i]=paa
            print(pati)
            sum=0
            for n in range(len(xj)-1):  
                sum += math.sqrt((xj[n] - xj[n+1]) ** 2 + (yj[n] - yj[n+1]) ** 2 + (zj[n] - zj[n+1])**2) 
                print(sum)        
            nm[i,j]=sum
            nm[j,i]=sum
            print('cost',nm)
    for i in range(len(x)):
        for j in range(len(x)):
            ppa=[]
            if nm[i,j]==0:
               nm[i,j]=math.sqrt((x[j] - x[i]) ** 2 + (y[j] - y[i]) ** 2 + (z[j] - z[i])**2)
               if i!=j:
                  ppa.append((x[i],y[i],z[i]))
                  ppa.append((x[j],y[j],z[j]))
               else:
                  ppa.append((x[j],y[j],z[j]))
               pati[i,j]=ppa
    print(nm)       
    pati=np.array(pati)
    pati=np.reshape(pati, (len(points),len(points)))
    print('pati',pati.shape)
    print('path',pati)
    np.fill_diagonal(nm, [np.inf])
    main(nm,hovtrans,points,cities,rr,rc,rd,rx,ry,rz,pati)
    #ant_colony = AntColony(nm, 20, 1, 500, 0.95, alpha=1, beta=1)
    #shortest_path = ant_colony.run()
    #print ("shorted_path: {}".format(shortest_path))
if __name__ == '__main__':
    dmain()
