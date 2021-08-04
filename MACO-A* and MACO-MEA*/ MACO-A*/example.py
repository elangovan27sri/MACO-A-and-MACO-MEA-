import numpy as np
#from plot import plot
import math
import csv
import matplotlib.pyplot as plt
from ant_colony import AntColony
from sympy import *
from obst import astar
from sympy import roots, solve_poly_system
def distance(city1: dict, city2: dict):
    return math.sqrt((city1['x'] - city2['x']) ** 2 + (city1['y'] - city2['y']) ** 2 + (city1['z'] - city2['z']) ** 2)
def main(maze,nm,hovtrans,points,cities,rr,rc,rd,rx,ry,rz,pati):
    
#system modelling
    den=1.225 #density of air kg/m3
    airsd=3.575 #airspeed m/s
    gracons=9.81 #gravitational const m/s
    bodywt=1.140 #drone body weight kg
    batwt=0.520 #battery weight kg

    pitang=5*(math.pi/180) #pitch angle in rad
    norot=4 #number of rotors
    grnspd=3 #average ground speed 3m/s
    dia=0.24 #propeller diameter in m
    print('pitch angle(in rad):',pitang)


# In[21]:


#for drag force
    CdA=0.48367 #drag coefficient
    drag=0.5*den*airsd**2*CdA
    print('drag force(in newton):',drag)


# In[22]:


#for thrust
    thr=(0.1019*((batwt+bodywt)*gracons+drag))*9.81 #kg
    print('thrust(newton):',thr)
 

# In[23]:


#for induced velocity
#ind=(2*thr)/((norot*dia**2*math.pi*den)*math.sqrt((grnspd*math.cos(pitang))**2+(grnspd*math.sin(pitang)+ind)**2))
    x = symbols('x')
    
    coef1=(2*thr)/(norot*dia**2*math.pi*den)
    coef2=(grnspd*math.cos(pitang))**2
    coef3=grnspd*math.sin(pitang)
    hg=solve(x**2*(coef2+(coef3+x)**2)-coef1**2, x)
    roots=hg[:2]
    ind = list(filter(lambda roots: roots >0, roots))
    ind =float(ind[0])
    print('induced velocity(m/s):',ind)


# In[24]:


#flying energy
    flyenemin=(ind+(grnspd*math.sin(pitang)))*thr #minimum energy for flying in J/s
    eff=0.7 #battery efficiency while travelling
    enefly=(flyenemin/eff)*(1/grnspd) #actual energy for flying..for distance D(m) energy is enefly*D
    print('minimum energy for flying(enefly*distance(m)):',enefly)


# In[25]:


#hovering energy
    hovenemin=(thr*math.sqrt(thr))/(math.sqrt(0.5*math.pi*norot*dia**2*den))
    efh=1.19 #battery efficiency while hovering
    enehov=hovenemin/efh ##actual energy for hovering..for time S(sec) energy is enehov*S
    print('minimum energy for hovering(enehov*hold(s)):',enehov)

    


    #cities = []
    #points = []
    #hovtrans= []
    #with open('./data/3.txt') as f:
     #   for line in f.readlines():
      #      city = line.split(' ')
       #     cities.append(dict(index=int(city[0]), x=int(city[1])/100, y=int(city[2])/100,z=int(city[3])/100,h=int(city[4])))
        #    points.append((int(city[1])/100, int(city[2])/100, int(city[3])/100))
        #    hovtrans.append(int(city[4]))
    #print('packet siz: transmission',hovtrans)
    #cost_matrix = []
    #rank = len(cities)
    #for i in range(rank):
     #   row = []
      #  for j in range(rank):
       #     row.append(ene*distance(cities[i], cities[j]))
       # cost_matrix.append(row)
    #am = np.array(cost_matrix)
    #np.fill_diagonal(am, [np.inf,np.inf,np.inf])
    #print('cost matrix',am)
    #ant_colony = AntColony(am, 20, 1, 100, 0.95, alpha=1, beta=1)
    #shortest_path = ant_colony.run()
    #print ("shorted_path: {}".format(shortest_path))
    #shor=np.array([shortest_path[0]])
    #sh=np.reshape(shor, (1,2*(rank)))
    #lh=sh[0,:]
    #print('path',lh)
    #cos=np.array([shortest_path[1]])
    #print('total energy for flying',cos)
    #pat=np.array([lh]).tolist()
    #path=pat[0]
    #tr=[]
    #xg=[]
    #la=[]
    #lb=[]
    #lc=[]
    #print('points',points)
   # plot(points, path,tr,xg,la,lb,lc)
#hovering
    cost_matrix_hov = []
    rank = len(cities)
    for i in range(rank):
        row = []
        for j in range(rank):
            row.append(hovtrans[j]*enehov)
        cost_matrix_hov.append(row)
    cost_hov=np.array(cost_matrix_hov)
    np.fill_diagonal(cost_hov, [np.inf,np.inf,np.inf])

    
    cost_fly=nm*enefly
    cost=cost_fly+cost_hov
    print('cost fly',cost_fly)
    print('cost hov',cost_hov)
    print('combined cost matrix',cost)
    print('distance between points',nm)
    ant_colony = AntColony(cost, 20, 1, 500, 0.95, alpha=1, beta=1)
    shortest_path = ant_colony.run()
    print ("shorted_path: {}".format(shortest_path))
    shor=np.array([shortest_path[0]])
    print('rank',rank)
    sh=np.reshape(shor, (1,2*(rank-1)))
    lh=sh[0,:]
    print('path',lh)
    cos=np.array([shortest_path[1]])
    print('total energy for all three',cos)
    pat=np.array([lh]).tolist()
    path=pat[0]
    print(path)
#for battery percentage cal
    bat=5200#in mAh
    bat=5200*60*60*0.001
    batv=16.8#in V
    baten=bat*batv#battery energy in joules
    print('battery energy capacity in joules',baten)
    sp=shortest_path[0]
    kk=[]
    for hw in sp:
        kk.append(cost_hov[hw])
    re=[]
    print('hovering',kk)
    print('overall hovering energy',sum(kk))
    print('overall flying energy till RTL', cos-sum(kk))
    print('overall % energy consumed till RTL',(cos/baten)*100)
    for mm in kk:
        re.append((mm/baten)*100)
    #print('energy used as per path:',sp,re)
    m0=0
    ht=[]
    for ss in re:
        hy=ss+m0
        m0=hy
        ht.append(m0)
    xg=[]
    for sw in ht:
        xg.append(100-sw)
    mh=np.reshape(shor, (rank-1,2))
    mh=mh[:,1]
    print('battery remaining  as per path node(%):',mh,xg)
    ml=[]
    for gt in mh:
        ml.append(points[gt])
    la,lb,lc = map(list,zip(*ml))# to get x,y,z alone
    print(la,lb,lc)
    pathi=[]
    for j in range(len(points)-1):
        pathi.append(pati[sp[j]])
    print('path by steps',pathi)
    plot(maze,pathi,rr,rc,rd,rx,ry,rz,xg,la,lb,lc,points,hovtrans)
import operator
import matplotlib.pyplot as plt
import numpy as np
import csv

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
def plot(maze,pathi,rr,rc,rd,rx,ry,rz,xg,la,lb,lc,points,hovtrans):
    x = []
    y = []
    z = []
    gs=len(pathi)+1
    print('gh',pathi)
    for i in range(len(pathi)):
        k=pathi[i]
        for m in k:
            x.append(m[0])
            y.append(m[1])
            z.append(m[2])
    print('xyz',x,y,z)
    #ax.scatter(x,y,z,'.')
    jj=[]
    for ss in xg:
        jj.append(round(ss, 2))
    print('ghffy',jj)
    cx=[]
    cy=[]
    cz=[]
    for i in points:
        cx.append(i[0])
        cy.append(i[1])
        cz.append(i[2])
    indexx=[]
    for j in range(len(points)):
        indexx.append(j)
 

    for i, txt in enumerate(indexx):
        if i>=1:
           ax.text(cx[i],cy[i],cz[i],'  {}'.format(txt),color='r',size=20)
    ax.text(x[0], y[0], 0, '   H',color='r',size=20)
    ax.scatter(x[0], y[0], 0,alpha=1, c='g', marker='s',s=150)
    ax.text(cx[0],cy[0],cz[0],'  0',color='r',size=20)
    ax.scatter(la,lb,lc,alpha=1,c='g',s=100,marker='o')
    ax.scatter(x[0],y[0],z[0],alpha=1,c='g',s=100,marker='o')
    for i in range(len(pathi)):
        print('elan',pathi[i])
        #ax.quiver(pathi[i][-1][0], pathi[i][-1][1],pathi[i][-1][2],pathi[i][-1][0]-pathi[i][-2][0], pathi[i][-1][1]-pathi[i][-2][1], pathi[i][-1][2]-pathi[i][-2][2], color='red',arrow_length_ratio=5,normalize=True)
        print(pathi[i][-1],pathi[i][-2])
    for i in range(len(rr)):
        rr[i]
        rr[i]+rx[i]
        X, Y, Z = np.meshgrid([rr[i],rr[i]+rx[i]],[rc[i],rc[i]+ry[i]],[rd[i],rd[i]+rz[i]])
  
        ax.plot_surface(X[0],Y[0],Z[0], alpha=1,color='b',edgecolor ='k',linewidth=1)
        ax.plot_surface(X[1],Y[1],Z[1], alpha=1,color='gray',edgecolor ='k',linewidth=1)
        Xi=np.zeros((2,2))
        Xi[0,:]=X[0][0,:] 
        Xi[1,:]=X[1][0,:]
        Yi=np.zeros((2,2))
        Yi[0,:]=Y[0][0,:] 
        Yi[1,:]=Y[1][0,:]
        ax.plot_surface(Xi,Yi,Z[0], alpha=1,color='k',edgecolor ='k',linewidth=1)
        Xj=np.zeros((2,2))
        Xj[0,:]=X[0][1,:] 
        Xj[1,:]=X[1][1,:]
        ax.plot_surface(Xj,Yi,Z[0], alpha=1,color='b',edgecolor ='k',linewidth=1)
        Zi=np.zeros((2,2))
        Zi[:,0]=Z[0][:,0] 
        Zi[:,1]=Z[1][:,0]
        ax.plot_surface(X[0],Yi.T,Zi, alpha=1,color='c',edgecolor ='k',linewidth=1)
        Zj=np.zeros((2,2))
        Zj[:,0]=Z[0][:,1] 
        Zj[:,1]=Z[1][:,1]
        ax.plot_surface(X[0],Yi.T,Zj, alpha=1,color='r',edgecolor ='k',linewidth=1)
                                    
    ax.set_xlabel('X',fontsize=30)
    ax.set_ylabel('Y',fontsize=30)
    ax.set_zlabel('Z',fontsize=30)
    #print('stu',s,t,u)
    ax.plot3D(x,y,z,'g',linewidth=4)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    #plt.zticks(fontsize=10)

    s=[]
    t=[]
    u=[]
    
    s.extend((x[0],x[0]))
    t.extend((y[0],y[0]))
    u.extend((0,z[0]))
    takdist=math.sqrt((x[0] - x[0]) ** 2 + (y[0] - y[0]) ** 2 + (0 - z[0]) ** 2)
    print('stu',s,t,u)
    sta=(x[-1],y[-1],z[-1])
    en=(s[1],t[1],u[1])
    ax.plot3D(s,t,u,'g',linewidth=4)
    pathee=astar(maze, sta, en)
    print('pathee',pathee)
    RTLdistn=[]
    for i in range(len(pathee)-1):
        j=pathee[i]
        k=pathee[i+1]
        RTLdistn.append(math.sqrt((j[0] - k[0]) ** 2 + (j[1] - k[1]) ** 2 + (j[2] - k[2]) ** 2))
    #print('rtl distance',sum(RTLdistn))
    m=[]
    n=[]
    o=[]
    for i in range(len(pathee)):
        m.append(pathee[i][0])
        n.append(pathee[i][1])
        o.append(pathee[i][2])
    ax.plot3D(m,n,o,'g',linewidth=4)
    ax.xaxis._axinfo["grid"]['linestyle'] = "-."
    ax.yaxis._axinfo["grid"]['linestyle'] = "-."
    ax.zaxis._axinfo["grid"]['linestyle'] = "-."
    
    patt=[]
    for i in range(len(pathi)):
        m=pathi[i]
        for j in range(len(m)):
            patt.append(m[j])   
    #ax.legend()
    print('for creating wavepoints',patt)
    
   
    distn=[]
    for i in range(len(patt)-1):
        j=patt[i]
        k=patt[i+1]
        distn.append(math.sqrt((j[0] - k[0]) ** 2 + (j[1] - k[1]) ** 2 + (j[2] - k[2]) ** 2))
    al=(2*takdist)+sum(distn)+sum(RTLdistn)
    print('overall distance travelled(meter):',al)
    print('overall flying energy(J):',al*62.56)  
    print('overall hovering time(seconds):',sum(hovtrans))
    print('overall hovering energy(J):',113.42*sum(hovtrans))
    print('overall energy(J):',(al*62.56)+(113.42*sum(hovtrans)))
    print('overall simulation flight time(seconds):',sum(hovtrans)+(al/2))#2m/s speed
    for i in range(len(pathee)-1):
        patt.append(pathee[i])
    print('dfd',patt)
    with open('path.txt', 'w') as f:
        csv.writer(f, delimiter=' ').writerows(patt)
    
    plt.show()
if __name__ == '__main__':
    main(maze,nm,hovtrans,points,cities,rr,rc,rd,rx,ry,rz,pati)

