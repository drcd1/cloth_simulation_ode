#ifndef NODE_SOLVERS
#define NODE_SOLVERS

#include <iostream>
#include "ode.h"
class ODESolver{

public:
    ODESolver(const ODE_func* f): f(f){}
    virtual ~ODESolver(){}
    virtual std::vector<std::vector<float>> solve(const std::vector<float>& x_ini,
                float t_ini,int steps,int substeps,float stepsize) const  = 0;
    const ODE_func* f;
};

class ExplicitEuler: public ODESolver{
public:
    ExplicitEuler(const ODE_func* f): ODESolver(f){}
    std::vector<std::vector<float>> solve(const std::vector<float>& x_ini,
                float t_ini,int steps,int substeps,float stepsize) const {

        std::vector<std::vector<float>> sol;
        sol.push_back(x_ini);
        int total_steps = steps*substeps;
        float tau = stepsize/substeps;
        std::vector<float> tmp = x_ini;
        std::vector<float> acc(x_ini.size());
        
        float time = t_ini;
        
            
        for(int k = 1; k<total_steps; k++){
            f->eval(tmp,time,&acc);
            for(int i = 0; i<tmp.size(); i++){
                tmp.at(i) = tmp.at(i) + acc.at(i)*tau;
            }
            if(k%substeps==0){
                sol.push_back(tmp);
            }
            time+=tau;
        }
        return sol;                    
    }
};
class ImplicitEuler: public ODESolver{
    int max_iter;
    float min_err_sqr;
public:
    ImplicitEuler(const ODE_func* f,int max_iter = 10,float min_err = 0.01): ODESolver(f), max_iter(max_iter),min_err_sqr(min_err*min_err){}
    std::vector<std::vector<float>> solve(const std::vector<float>& x_ini,
                float t_ini,int steps,int substeps,float stepsize) const {
        
        std::vector<std::vector<float>> sol;
        sol.push_back(x_ini);
        int total_steps = steps*substeps;
        float tau = stepsize/substeps;
        std::vector<float> tmp = x_ini;
        std::vector<float> tmp2 = x_ini;
        std::vector<float> old_tmp2 = x_ini;
        std::vector<float> acc(x_ini.size());
        
        float time = t_ini;
        
            
        for(int k = 1; k<total_steps; k++){
            tmp2 = tmp;
            for(int m = 0; m<max_iter;m++){
                old_tmp2 = tmp2;
                f->eval(tmp2,time,&acc);                
                for(int i = 0; i<tmp.size(); i++){
                    tmp2.at(i) = tmp.at(i) + acc.at(i)*tau;
                }
                float err_sqr = 0;
                for(int i = 0; i<tmp2.size();i++){
                    err_sqr += (tmp2.at(i)-old_tmp2.at(i))*(tmp2.at(i)-old_tmp2.at(i));
                }
                if(err_sqr<min_err_sqr){
                    break;
                }
            }
            for(int i = 0; i<tmp.size(); i++){
                tmp.at(i) = tmp.at(i) + acc.at(i)*tau;
            }
            
            if(k%substeps==0){
                sol.push_back(tmp);
            }
            time+=tau;
        }
        return sol;                    
    }
};
class Collision{
    public:
        bool test;
        Vec3 n;
};

class Capsule {
    private:
        Vec3 a,b;
        float r;
    public:
        Capsule(Vec3 a, Vec3 b, float r):a(a),b(b),r(r){
            
        }
        Collision testCollision(Vec3 x){
            Collision c;
            Vec3 n(b-a);
            float t = (x-a).dot(n)/(n).lenSqr();
            

            //clamp t
            t= (t<0?0:(t>1?1:t));
            Vec3 closestPoint(a+n*t);
            float eps = 0.01;
            
            /*if(t<1){
                std::cout<<t<<std::endl;
                std::cout<<closestPoint.x<<" "<<closestPoint.y<<" "<<closestPoint.z<<std::endl;
            }*/
            if((r+eps)*(r+eps)>(x-closestPoint).lenSqr()){
                c.test = true;
                c.n = (x-closestPoint).normalized();
            } else {
                
                c.test = false;
                c.n = Vec3(0,0,1);
            }
            return c;
            
        }
};
class SymplecticEuler: public ODESolver{
public:
    SymplecticEuler(const ODE_func* f): ODESolver(f){}
    std::vector<std::vector<float>> solve(const std::vector<float>& x_ini,
                float t_ini,int steps,int substeps,float stepsize) const {
        
        std::vector<std::vector<float>> sol;
        sol.push_back(x_ini);
        int total_steps = steps*substeps;
        float tau = stepsize/substeps;
        std::vector<float> tmp = x_ini;
        std::vector<float> acc(x_ini.size());
        /*
        Capsule caps(Vec3(-5,0,-0.5),Vec3(5,0,-0.5),0.4);
        Capsule caps2(Vec3(0,-5,-0.5),Vec3(0,5,-0.5),0.4);
        */  
        float time = t_ini;
        
            
        for(int k = 1; k<total_steps; k++){
            f->eval(tmp,time,&acc);
            int v = tmp.size()/2;

            //update only velocities first
            for(int i = v; i<tmp.size(); i++){
                tmp.at(i) = tmp.at(i) + acc.at(i)*tau;
            }

            /*//at the end of the timestep, correct for collisions
            for(int i = 0; i<v/3;i++){
                Vec3 x(tmp.at(i*3),tmp.at(i*3+1),tmp.at(i*3+2));
                Vec3 vel(tmp.at(i*3+v),tmp.at(i*3+v+1),tmp.at(i*3+v+2));
                Collision c = caps.testCollision(x);
                if(c.test){
                    if(c.n.dot(vel)<0){
                        vel = vel - c.n*c.n.dot(vel);
                        tmp.at(i*3+v) = vel.x;
                        tmp.at(i*3+v+1)= vel.y;
                        tmp.at(i*3+v+2) = vel.z;
                    }
                }
                c = caps2.testCollision(x);
                if(c.test){
                    if(c.n.dot(vel)<0){
                        vel = vel - c.n*c.n.dot(vel);
                        tmp.at(i*3+v) = vel.x;
                        tmp.at(i*3+v+1)= vel.y;
                        tmp.at(i*3+v+2) = vel.z;
                    }
                }
                

            }*/

            //update only positions
            for(int i = 0; i<v; i++){
                tmp.at(i) = tmp.at(i) + tmp.at(i+v)*tau;
            }
            if(k%substeps==0){
                sol.push_back(tmp);
            }

            


            time+=tau;
        }
        return sol;                    
    } 

};

class RungeKutta4: public ODESolver{
private:
    float c2, c3,c4,a21,a32,a43,b1,b2,b3,b4;
public:
    RungeKutta4(const ODE_func* f): ODESolver(f),c2(0.5),c3(0.5),c4(1),a21(0.5),a32(0.5),a43(1),b1(1.0f/6),b2(1.0f/3),b3(1.0f/3),b4(1.0f/6){}
    std::vector<std::vector<float>> solve(const std::vector<float>& x_ini,
                float t_ini,int steps,int substeps,float stepsize) const {
        
        std::vector<std::vector<float>> sol;
        sol.push_back(x_ini);
        int total_steps = steps*substeps;
        float tau = stepsize/substeps;
        std::vector<float> tmp = x_ini;
        
        std::vector<float> tmp2(x_ini.size());
        std::vector<float> h1(x_ini.size());
        std::vector<float> h2(x_ini.size());
        std::vector<float> h3(x_ini.size());
        std::vector<float> h4(x_ini.size());
        
        float time = t_ini;
        
            
        for(int k = 1; k<total_steps; k++){
            f->eval(tmp,time,&h1);
            for(int i = 0; i<tmp.size(); i++){
                tmp2.at(i) = tmp.at(i)+a21*tau*h1.at(i);
            }
            f->eval(tmp2,time+c2*tau,&h2);
            for(int i = 0; i<tmp.size(); i++){
                tmp2.at(i) = tmp.at(i)+a32*tau*h2.at(i);
            };            
            f->eval(tmp2,time+c3*tau,&h3);
            for(int i = 0; i<tmp.size(); i++){
                tmp2.at(i) = tmp.at(i)+a43*tau*h3.at(i);
            };
            f->eval(tmp2,time+c4*tau,&h4);

            for(int i = 0; i<tmp.size(); i++){
                tmp.at(i) = tmp.at(i) + (h1.at(i)*b1 + h2.at(i)*b2 + h3.at(i)*b3 + h4.at(i)*b4)*tau;
            }
            if(k%substeps==0){
                sol.push_back(tmp);
            }
            time+=tau;
        }
        return sol;                    
    }

};

//Adams Method with 4 steps
class Adams4: public ODESolver{
private:
    float b1,b2,b3,b4;
public:
    Adams4(const ODE_func* f): ODESolver(f){
        b1 = 2.29166666667;
        b2 = -2.4583333333;
        b3 = 1.54166666667;
        b4 = -0.375;     
    }
    std::vector<std::vector<float>> solve(const std::vector<float>& x_ini,
                float t_ini,int steps,int substeps,float stepsize) const {
        std::vector<std::vector<float>> sol;
        sol.push_back(x_ini);
        int total_steps = steps*substeps;
        float tau = stepsize/substeps;
        std::vector<float> tmp = x_ini;
        
        float time = t_ini;
        //compute the first 4 steps with RK
        //note: due to the structure of the code, there is some slight overhead here,
        // but it only happens for the first 4 steps    
        
        std::vector<std::vector<float>> tmpSol;
        {
        RungeKutta4 rk4(f);
        tmpSol = rk4.solve(x_ini,t_ini,4,1,tau);
        }

        std::vector<std::vector<float>> previousF;
        for(int i = 0; i<4; i++){
            previousF.push_back(std::vector<float>(x_ini.size()));
            f->eval(tmpSol.at(i),time,&(previousF.at(i)));
            time +=tau;
        }
        for(int i = 1; i<4; i++){
            if(i%substeps == 0){
                sol.push_back(tmpSol.at(i));
            }
        }

        for(int k = 4; k<total_steps; k++){
            for(int i = 0; i<tmp.size(); i++){
                tmp.at(i) = tmp.at(i) + ((b1*previousF.at((k-1)%4).at(i)) + 
                                         (b2*previousF.at((k-2)%4).at(i)) + 
                                         (b3*previousF.at((k-3)%4).at(i)) + 
                                         (b4*previousF.at((k-4)%4).at(i)))*tau;
            }
            if(k%substeps==0){
                sol.push_back(tmp);
            }            
            time+=tau;            
            f->eval(tmp,time,&(previousF.at(k%4)));
        }
        return sol;          
    }
                    
};

class StormerVerlet: public ODESolver{
public:
    StormerVerlet(const ODE_func* f): ODESolver(f){}
    std::vector<std::vector<float>> solve(const std::vector<float>& x_ini,
                float t_ini,int steps,int substeps,float stepsize) const {
        
        std::vector<std::vector<float>> sol;
        sol.push_back(x_ini);
        int total_steps = steps*substeps;
        float tau = stepsize/substeps;
        std::vector<float> tmp = x_ini;
        std::vector<float> acc(x_ini.size());
        
        float time = t_ini;
        //Note: we evaluate the points on a staggered grid in time:
        // the velocity is always half a step ahead
        
        f->eval(tmp,time,&acc);
        int v = tmp.size()/2;

        //update only velocities first
        for(int i = v; i<tmp.size(); i++){
            tmp.at(i) = tmp.at(i) + acc.at(i)*(tau*0.5);
        }
            
        for(int k = 1; k<total_steps; k++){

            //update only positions
            for(int i = 0; i<v; i++){
                tmp.at(i) = tmp.at(i) + tmp.at(i+v)*tau;
            }
            
            f->eval(tmp,time,&acc);
            int v = tmp.size()/2;

            //update only velocities
            for(int i = v; i<tmp.size(); i++){
                tmp.at(i) = tmp.at(i) + acc.at(i)*tau;
            }
            

            if(k%substeps==0){
                sol.push_back(tmp);
            }

            time+=tau;
        }
        return sol;                    
    } 

};

#endif