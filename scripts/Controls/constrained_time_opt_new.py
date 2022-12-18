from math import factorial as f
import numpy as np
from scipy.linalg import block_diag
from qpsolvers import solve_qp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import warnings
import tensorflow as tf
from scipy.optimize import Bounds,minimize
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)
warnings.filterwarnings("ignore", category=FutureWarning)
warnings.filterwarnings("ignore")
class min_snap:
    def __init__(self, x, y, z, v,v_min,v_max,n=8):        
        self.v = v
        self.v_max=v_max
        self.v_min=v_min
        self.n = n
        self.m = len(x)-1
        m = self.m
        
        self.x = x
        self.y = y
        self.z = z

        self.K_t=15
        self.acc_upper_bound=0.4
        self.scale_fac=0.5

        self.start_time=0.1
        self.t_test = self.time_array(self.v)
        self.t =np.copy(self.t_test)
        self.t_interval=self.give_intervals(self.t)
        print('\n\nTotal Time - TEST CASE :',self.give_tot(self.t_interval))
        
        self.t_intervals_min=self.give_intervals(self.time_array(self.v_max))
        #print('Max Velocity Possible :',self.v_max,'| Total time - MIN :',self.give_tot(self.t_intervals_min))
        #print(self.t_intervals_min)
        

        self.t_intervals_max=self.give_intervals(self.time_array(self.v_min))
        #print('Min Velocity Allowed :',self.v_min,'| Total time - MAX :',self.give_tot(self.t_intervals_max))
        #print(self.t_intervals_max)



        self.q=np.zeros(shape=(n*m,1)).reshape((n*m,))
        self.G= np.zeros(shape=(3,n*m))  
        self.G_y=np.copy(self.G)                              #np.zeros(shape=((4*m)+2,n*m))
        self.h= np.zeros(shape=(3,))   
        self.h_y=np.copy(self.h)                           #np.zeros(shape=((4*m)+2,1)).reshape(((4*m)+2,))
        

        b_x = np.array([x[0],0,0,x[m],0,0])

        b_x = np.append(b_x, x[1:m])
        b_x = np.append(b_x, np.zeros(shape=(3*(m-1))))
        b_y = np.array([y[0],0,0,y[m],0,0])
        b_y = np.append(b_y, y[1:m])
        b_y = np.append(b_y, np.zeros(shape=(3*(m-1))))
        b_z = np.array([z[0],0,0,z[m],0,0])
        b_z = np.append(b_z, z[1:m])
        b_z = np.append(b_z, np.zeros(shape=(3*(m-1))))
        self.b_x = b_x
        self.b_y = b_y
        self.b_z = b_z
        #self.form_Q()
        #self.form_A()
        self.p_x=0
        self.p_y=0
        self.p_z=0
        self.J=0

        self.x_path=[]
        self.x_dot_path=[]
        self.x_dot_dot_path=[]

        self.y_path=[]
        self.y_dot_path=[]
        self.y_dot_dot_path=[]

        self.z_path=[]
        self.z_dot_path=[]
        self.z_dot_dot_path=[]

        self.dt=0.1


    def time_array(self,v):
        t = [self.start_time]
        for i in range(1,self.m+1):
            dist = np.sqrt((self.x[i]-self.x[i-1])**2 + (self.y[i]-self.y[i-1])**2 + (self.z[i]-self.z[i-1])**2)
            ti = dist/v
            t.append(t[-1]+ti)
        return t

    def give_tot(self,t_intervals):
        t_calc=[self.start_time]
        sum=self.start_time
        for i in range(len(t_intervals)):
            sum+=t_intervals[i]
            t_calc.append(sum)
        return t_calc[-1]-t_calc[0]


    def form_Q(self,t):
        Q_list = []
        for l in range(1,self.m+1):
            
            Q_i=np.zeros(shape=(self.n,self.n))
            for i in range(self.n):       
                for j in range(self.n):
                    if (((i>3) and (j<=3))) or (((j>3) and (i<=3))):
                        Q_i[i][j]=0
                    elif (i<=3) and (j<=3):
                        Q_i[i][j]=0
                    else:
                        r,c=i+1,j+1
                        Q_i[i][j]= (f(r)*f(c)*(pow(t[l],r+c-7)-pow(t[l-1],r+c-7)))/(f(r-4)*f(c-4)*(r+c-7))
            Q_list.append(Q_i)
        Q = Q_list[0]
        Q_list.pop(0)
        for i in Q_list:
            Q=block_diag(Q,i)
        self.Q=Q+(0.0001*np.identity(self.n*self.m))
        #print(type(self.Q),'typeofQ')

    def form_A(self,t):
        n = self.n
        m = self.m
        #t = self.t
        A=np.zeros(shape=((4*m)+2,n*m))

        for j in range(n*m):
                if j>=n:
                    A[0][j],A[1][j],A[2][j]=0,0,0
                else:
                    A[0][j],A[1][j],A[2][j]=pow(t[0],j),j*pow(t[0],j-1),j*(j-1)*pow(t[0],j-2)

        for j in range(n*m):
                if j<n*(m-1):
                    A[3][j],A[4][j],A[5][j]=0,0,0
                else:
                    h=n*(m-1)
                    A[3][j],A[4][j],A[5][j]=pow(t[m],j-h),(j-h)*pow(t[m],j-h-1),(j-h)*(j-h-1)*pow(t[m],j-h-2)
        z=[]
        for i in range(1,m):
            h=[]
            for j in range(n*m):
                if (j<((i-1)*n)) or (j>=(i*n)):
                    h.append(0)
                else:
                    h.append(pow(t[i],j-((i-1)*n)))
            z.append(h)    
        A[6:(6+m-1)]=z
        pva_const=[]
        for i in range(1,m):
            x_i,v_i,a_i=[],[],[]
            for j in range(n*m):
                if (j<((i-1)*n)) or (j>=((i+1)*n)):
                    x_i.append(0)
                    v_i.append(0)
                    a_i.append(0)
                    
                elif (j<((i)*n)) and (j>=((i-1)*n)):
                    x_i.append(pow(t[i],j-((i-1)*n)))
                    v_i.append((j-((i-1)*n))*pow(t[i],j-1-((i-1)*n)))
                    a_i.append((j-1-((i-1)*n))*(j-((i-1)*n))*pow(t[i],j-2-((i-1)*n)))
                else:
                    x_i.append((-1)*pow(t[i],j-((i)*n)))
                    v_i.append((-1)*(j-((i)*n))*pow(t[i],j-1-((i)*n)))
                    a_i.append((-1)*(j-1-((i)*n))*(j-((i)*n))*pow(t[i],j-2-((i)*n)))
            pva_i=[x_i,v_i,a_i]        
            pva_const=pva_const+pva_i
        A[(6+m-1):]=pva_const
        self.A = A
        #print(type(self.A),'typeofA')

    def form_G(self,t):
        l=4
        n=self.n
        for i in range(((l-1)*n),(l*n)):
            indx=i-((l-1)*n)
            self.G_y[0,i]=-1*indx*(indx-1)*(pow(t[l+1],indx-2))
        l=7
        for i in range(((l-1)*n),(l*n)):
            indx=i-((l-1)*n)
            self.G_y[1,i]=indx*(indx-1)*(pow(t[l+1],indx-2))
        l=10
        for i in range(((l-1)*n),(l*n)):
            indx=i-((l-1)*n)
            self.G_y[2,i]=-1*indx*(indx-1)*(pow(t[l+1],indx-2))




    def form_h(self,t):
        self.h_y[0],self.h_y[1],self.h_y[2]=self.acc_upper_bound,self.acc_upper_bound,self.acc_upper_bound



         

    def solve(self):
        #print(type(self.q),type(self.G),type(self.h),type(self.b_x))
        self.p_x=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_x)
        self.p_y=solve_qp(self.Q, self.q,self.G_y,self.h_y, self.A, self.b_y)
        self.p_z=solve_qp(self.Q, self.q,self.G,self.h, self.A, self.b_z)
        
    
    def cost_func(self,t_input):
        t=[self.start_time]
        sum=self.start_time
        for i in range(len(t_input)):
            sum+=t_input[i]
            t.append(sum)
        self.form_Q(t)
        self.form_A(t)
        self.form_G(t)
        self.form_h(t)

        K=self.K_t
        self.solve()
        p_x=self.p_x
        p_y=self.p_y
        p_z=self.p_z
        Q=self.Q
        #t=np.copy(self.t)
        self.J=(self.scale_fac*(np.matmul(np.matmul(np.transpose(p_x),Q),p_x))) +(self.scale_fac*(np.matmul(np.matmul(np.transpose(p_y),Q),p_y)))+(self.scale_fac*(np.matmul(np.matmul(np.transpose(p_z),Q),p_z)))+ (K*(t[-1]-t[0]))
        return self.J
    
    def plot_test_case(self,colr,label):
        cost_test_case=self.cost_func(self.t_interval)
        self.plot(colr,label)
        


    def plot(self,colr,label):
        #ax = plt.axes(projection ='3d')

        ax.scatter(self.x, self.y, self.z, c='black',marker='o',s=20)

        for v in range(self.m):
            w,u,a=[],[],[]
            
            r=np.linspace(self.t[v],self.t[v+1],100)
            for i in range(100):
                g,e,f=0,0,0
                for j in range(self.n*v,(v+1)*self.n):
                    g=g+(self.p_x[j]*pow(r[i],j-(self.n*v)))
                    e=e+(self.p_y[j]*pow(r[i],j-(self.n*v)))
                    f=f+(self.p_z[j]*pow(r[i],j-(self.n*v)))
                w.append(g)
                u.append(e)
                a.append(f)
            ax.plot3D(w, u, a, colr)
           
        
        ax.plot3D(w, u, a, colr,label=label)
        #plt.show()

    def give_intervals(self,t):
        m=[]
        for j in range(len(t)-1):
            m.append(t[j+1]-t[j])
        return m

        

    def optimize(self):
        bounds = Bounds(self.t_intervals_min,self.t_intervals_max)
        x_bar=self.t_interval
        res = minimize(self.cost_func, x_bar, method='trust-constr',bounds=bounds)
        print('\nOptmization Success Status :',res.success)
        print('No : of iterations :',res.niter,'\n')
        #print(res)
        
        self.t_interval=res.x
        #print(self.t_interval)
        t_final=[self.start_time]
        sum=self.start_time
        for i in range(len(self.t_interval)):
            sum+=self.t_interval[i]
            t_final.append(sum)
        self.t=np.copy(t_final)
        #print("Optimized Time segmentation :",self.t)
        print('Optimized Total time :',self.t[-1]-self.t[0])

    def get_trajectory_var(self):
        for v in range(self.m):
            w,u,a=[],[],[]
            w_v,u_v,a_v=[],[],[]
            w_a,u_a,a_a=[],[],[]
            
            r=np.arange(self.t[v],self.t[v+1],self.dt)
            for i in range(0,r.shape[0]):
                g,g_v,g_a,e,e_v,e_a,f,f_v,f_a=0,0,0,0,0,0,0,0,0
                for j in range(self.n*v,(v+1)*self.n):
                    g=g+(self.p_x[j]*pow(r[i],j-(self.n*v)))
                    e=e+(self.p_y[j]*pow(r[i],j-(self.n*v)))
                    f=f+(self.p_z[j]*pow(r[i],j-(self.n*v)))

                    g_v=g_v+((j-(self.n*v))*self.p_x[j]*pow(r[i],j-1-(self.n*v)))
                    e_v=e_v+((j-(self.n*v))*self.p_y[j]*pow(r[i],j-1-(self.n*v)))
                    f_v=f_v+((j-(self.n*v))*self.p_z[j]*pow(r[i],j-1-(self.n*v)))

                    g_a=g_a+((j-(self.n*v))*(j-1-(self.n*v))*self.p_x[j]*pow(r[i],j-2-(self.n*v)))
                    e_a=e_a+((j-(self.n*v))*(j-1-(self.n*v))*self.p_y[j]*pow(r[i],j-2-(self.n*v)))
                    f_a=f_a+((j-(self.n*v))*(j-1-(self.n*v))*self.p_z[j]*pow(r[i],j-2-(self.n*v)))


                w.append(g)
                w_v.append(g_v)
                w_a.append(g_a)

                u.append(e)
                u_v.append(e_v)
                u_a.append(e_a)

                a.append(f)
                a_v.append(f_v)
                a_a.append(f_a)
  
            self.x_path.extend(w)
            self.x_dot_path.extend(w_v)
            self.x_dot_dot_path.extend(w_a)

            self.y_path.extend(u)
            self.y_dot_path.extend(u_v)
            self.y_dot_dot_path.extend(u_a)

            self.z_path.extend(a)
            self.z_dot_path.extend(a_v)
            self.z_dot_dot_path.extend(a_a)

            self.psi_path=np.arctan2(self.y_dot_path,self.x_dot_path)

        #print(self.x_path,'\n\n',self.x_dot_path,'\n\n',self.x_dot_dot_path,'\n\n',self.y_path,'\n\n',self.y_dot_path,'\n\n',self.y_dot_dot_path,'\n\n',self.z_path,'\n\n',self.z_dot_path,'\n\n',self.z_dot_dot_path,'\n\n',self.psi_path,'\n\n')

        return self.x_path,self.x_dot_path,self.x_dot_dot_path,self.y_path,self.y_dot_path,self.y_dot_dot_path,self.z_path,self.z_dot_path,self.z_dot_dot_path,self.psi_path

        #print(self.y_dot_path,'\n\n')
        #print(self.y_dot_dot_path)

            





            

        


if __name__ == '__main__':
 
    '''x=[-3,0,3,3,0,-3,-3,0,3,3,0,-3,-3,0]
    y=[-35,-25,-12.5,12.5,25,12.5,-12.5,-25,-12.5,12.5,25,12.5,-12.5,-25]
    z=[7,7,7,7,7,7,7,7,7,7,7,7,7,7]'''

    x=[0,10,0,-10,0,10,0,-10,0]
    y=[0,12,22,12,2,12,22,12,2]
    z=[7,7,7,7,7,7,7,7,7]
    v_test = 1
    v_min=0.1
    v_max=2

    plt.figure(figsize=(10,5))
    ax = plt.axes(projection ='3d')

    ms = min_snap(x,y,z,v_test,v_min,v_max)

    #55555ms.plot_test_case('r','Test Case Trajectory')

    ms.optimize()

    ms.get_trajectory_var()
    

    ms.plot('g','Time Optimized Trajectory')
    
    plt.legend()
    ax.set_zlim3d(0,15)
    ax.set_xlim3d(-40,40)
    ax.set_ylim3d(-50,50)
    
    plt.show()