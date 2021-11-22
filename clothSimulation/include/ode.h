#ifndef NODE_ODE
#define NODE_ODE
#include <vector>
#include <array>
#include <iterator>
#include <thread>
#include <chrono>
#include "../../renderer/include/math.h"

#define INV_PI  0.31830988618

class Force{
    public:
        virtual ~Force(){}
        virtual void dvdtAcc(const std::vector<float>& x, float t, const std::vector<float>& masses, std::vector<float>* acc) const =0;
        virtual void accU(const std::vector<float>& x, float t, const std::vector<float>& masses, float* e) const =0;
};

class CapsulePenaltyForceDamped: public Force{
    private:
        Vec3 a,b;
        float r;
        float threshold;
        float stiffness;
        float damping;
    public:
        CapsulePenaltyForceDamped(Vec3 a, Vec3 b, float r, float threshold, float stiffness, float damping):
            a(a),b(b),r(r),threshold(threshold), stiffness(stiffness), damping(damping){
        }
        //does not represent a potential (it has damping)
        virtual void accU(const std::vector<float>& x, float t, const std::vector<float>& masses, float* e) const {}
        void dvdtAcc(const std::vector<float>& x, float t, const std::vector<float>& masses, std::vector<float>* acc) const {
            int v = x.size()/2;
            for(int i = 0; i<masses.size();i++){
                Vec3 accel(0,0,0);
                
                Vec3 pos(x.at(i*3),x.at(i*3+1),x.at(i*3+2));
                Vec3 vel(x.at(i*3+v),x.at(i*3+1+v),x.at(i*3+2+v));

                Vec3 n(b-a);
                float t = (pos-a).dot(n)/(n).lenSqr();
            
                //clamp t
                t= (t<0?0:(t>1?1:t));
                
                Vec3 closestPoint(a+n*t);
                Vec3 to_pos = (pos-closestPoint);
                float dist_sqr = to_pos.lenSqr();
                if(dist_sqr>=(r+threshold)*(r+threshold)){
                    continue;
                }
                float dist = sqrt(dist_sqr);
                float l = dist-(r+threshold);
                Vec3 to_pos_normalized = to_pos/dist;
                //spring force
                accel = accel -  (to_pos_normalized)*l*stiffness/masses.at(i);
                //damping
                acc->at(i*3) +=accel.x;                
                acc->at(i*3 + 1) +=accel.y;                
                acc->at(i*3 + 2) +=accel.z;
            }
        }



};

class DampedSpringForce: public Force{
    private:
        int m1, m2;
        float k,l,damping;
    public:
        DampedSpringForce(int m1,int m2,float k, float l,float damping):m1(m1),m2(m2),k(k),l(l),damping(damping){}
        
        //does not represent a potential (it has damping)
        virtual void accU(const std::vector<float>& x, float t, const std::vector<float>& masses, float* e) const {}
        
        void dvdtAcc(const std::vector<float>& x, float t,const std::vector<float>& masses, std::vector<float>* acc) const {

          
            //get particle positions
            
            Vec3 p1(x.at(m1*3),x.at(m1*3+1),x.at(m1*3+2));
            Vec3 p2(x.at(m2*3),x.at(m2*3+1),x.at(m2*3+2));
            int v = x.size()/2;
            Vec3 vel1(x.at(m1*3+v),x.at(m1*3+1+v),x.at(m1*3+2+v));
            Vec3 vel2(x.at(m2*3+v),x.at(m2*3+1+v),x.at(m2*3+2+v));
            Vec3 n = (p2-p1);
            float dist = n.len();
            n = n/dist;
            float fac = k*(dist-l);
            float fac1 = 1.0f/masses.at(m1);
            float fac2 = 1.0f/masses.at(m2);
            
            //damping happens along the connection
            Vec3 accel1 = n*((fac-damping*vel1.dot(n))*fac1);
            Vec3 accel2 = n*((-fac-damping*vel2.dot(n))*fac2);
            acc->at(m1*3)+=accel1.x;
            acc->at(m1*3+1)+=accel1.y;
            acc->at(m1*3+2)+=accel1.z;
            acc->at(m2*3)+=accel2.x;
            acc->at(m2*3+1)+=accel2.y;
            acc->at(m2*3+2)+=accel2.z;
                   
        }
};
class GravityForce: public Force{
    private:
        Vec3 gravity;
    public:
        GravityForce(Vec3 gravity): gravity(gravity){}
        
        virtual void accU(const std::vector<float>& x, float t, const std::vector<float>& masses, float* e) const {
            for(int i = 0; i<masses.size();i++){
                *e += -x.at(i*3)*masses.at(i)*gravity.x -x.at(i*3+1)*masses.at(i)*gravity.y -x.at(i*3+2)*masses.at(i)*gravity.z;
            }
        }
        
        //x = (pos,v)
        // acc.size = 
        void dvdtAcc(const std::vector<float>& x, float t, const std::vector<float>& masses, std::vector<float>* acc) const {

            for(int i = 0; i<masses.size(); i++){             
                acc->at(i*3) += gravity.x;                
                acc->at(i*3 + 1) += gravity.y;                
                acc->at(i*3 + 2) += gravity.z;
            }
        }
};
class AirResistance: public Force{
private:
    std::vector<std::array<int,3>> triangles;
    float coef;
public:
    AirResistance(std::vector<std::array<int,3>> triangles, float coef):triangles(triangles),coef(coef){

    }
    //does not represent a potential
    virtual void accU(const std::vector<float>& x, float t, const std::vector<float>& masses, float* e) const {
    }

    void dvdtAcc(const std::vector<float>& x, float t, const std::vector<float>& masses, std::vector<float>* acc) const {
        for(auto t: triangles){
            Vec3 x1(x.at(t.at(0)*3),x.at(t.at(0)*3+1),x.at(t.at(0)*3+2));
            Vec3 x2(x.at(t.at(1)*3),x.at(t.at(1)*3+1),x.at(t.at(1)*3+2));
            Vec3 x3(x.at(t.at(2)*3),x.at(t.at(2)*3+1),x.at(t.at(2)*3+2));
            int v = x.size()/2;
            Vec3 vel1(x.at(v+t.at(0)*3),x.at(v+t.at(0)*3+1),x.at(v+t.at(0)*3+2));
            Vec3 vel2(x.at(v+t.at(1)*3),x.at(v+t.at(1)*3+1),x.at(v+t.at(1)*3+2));
            Vec3 vel3(x.at(v+t.at(2)*3),x.at(v+t.at(2)*3+1),x.at(v+t.at(2)*3+2));

            Vec3 n = (x2-x1).cross(x3-x2);

            //twice the area spaned by the triangle
            float area2 = n.len();
            
            //normalize
            n = n/area2;

            //multiply by the angle percentage
            Vec3 e1 = (x2-x1).normalized();
            Vec3 e2 = (x3-x1).normalized();
            float cosA1 = e1.dot(e2);
            float cosA2 = -e1.dot((x3-x2).normalized());

            float a1 = acos(cosA1)*INV_PI;
            float a2 = acos(cosA2)*INV_PI;
            float a3 = 1-a2-a1;
            
            //compute accelerations
            Vec3 acc1 = n*(-coef*a1*area2*0.5*vel1.dot(n));
            Vec3 acc2 = n*(-coef*a2*area2*0.5*vel2.dot(n));
            Vec3 acc3 = n*(-coef*a3*area2*0.5*vel3.dot(n));
          
            acc->at(t.at(0)*3) += acc1.x;
            acc->at(t.at(0)*3+1) += acc1.y;
            acc->at(t.at(0)*3+2) += acc1.z;

            acc->at(t.at(1)*3) += acc2.x;
            acc->at(t.at(1)*3+1) += acc2.y;
            acc->at(t.at(1)*3+2) += acc2.z;

            acc->at(t.at(2)*3) += acc3.x;
            acc->at(t.at(2)*3+1) += acc3.y;
            acc->at(t.at(2)*3+2) += acc3.z;



            
        }
    }
};

class SpringForce: public Force{
    private:
        int m1, m2;
        float k,l;
    public:
        SpringForce(int m1,int m2,float k, float l):m1(m1),m2(m2),k(k),l(l){}
        virtual void accU(const std::vector<float>& x, float t, const std::vector<float>& masses, float* e) const {
            Vec3 p1(x.at(m1*3),x.at(m1*3+1),x.at(m1*3+2));
            Vec3 p2(x.at(m2*3),x.at(m2*3+1),x.at(m2*3+2));
            Vec3 n = (p2-p1);
            float dist = n.len();
            n = n/dist;
            float fac = (dist-l);
            *e += 0.5*k*fac*fac;
        }
        void dvdtAcc(const std::vector<float>& x, float t,const std::vector<float>& masses, std::vector<float>* acc) const {
            //get particle positions
            Vec3 p1(x.at(m1*3),x.at(m1*3+1),x.at(m1*3+2));
            Vec3 p2(x.at(m2*3),x.at(m2*3+1),x.at(m2*3+2));
            Vec3 n = (p2-p1);
            float dist = n.len();
            n = n/dist;
            float fac = k*(dist-l);
            float fac1 = 1.0f/masses.at(m1);
            float fac2 = 1.0f/masses.at(m2);
            
            acc->at(m1*3)+=n.x*fac*fac1;
            acc->at(m1*3+1)+=n.y*fac*fac1;
            acc->at(m1*3+2)+=n.z*fac*fac1;
            acc->at(m2*3)-=n.x*fac*fac2;
            acc->at(m2*3+1)-=n.y*fac*fac2;
            acc->at(m2*3+2)-=n.z*fac*fac2;
            
                    
        }
};


//ODE_func: defines the right hand side of a first order ODE
// x' = f(x,t)
class ODE_func{
    public:
        //writes the result to x_prime
        virtual void eval(const std::vector<float>& x,float t, std::vector<float>* dxdt) const =0;
        
        //accumulates potential energy on e;
        virtual void potentialEnergy(const std::vector<float>& x, float t,float* e) const = 0;
        //accumulates kinectic energy on e;
        virtual void kinecticEnergy(const std::vector<float>& x, float t,float* e) const = 0;
        virtual ~ODE_func(){}
};

//applies a set of forces to a function
class Physics_func: public ODE_func{
    private:
        std::vector<Force*> forces;
        std::vector<int> fixed;
        std::vector<float> masses;
    public:
        Physics_func(const std::vector<Force*>& forces, const std::vector<int>& fixed, const std::vector<float>& masses): forces(forces),fixed(fixed), masses(masses){}
        ~Physics_func(){
            for(auto f: forces){
                delete f;
            }
        }
        void potentialEnergy(const std::vector<float>& x, float t,float* e) const {
            for(int i = 0; i<forces.size(); i++){
                forces.at(i)->accU(x,t,masses,e);
            }
        }
        void kinecticEnergy(const std::vector<float>& x, float t,float* e) const {
            for(int i = 0; i<masses.size(); i++){
                int v = x.size()/2;
                Vec3 vel(x.at(i*3+v),x.at(i*3+v+1),x.at(i*3+v+2));
                *e += 0.5*masses.at(i)*(vel.lenSqr());
            }
        }
        void eval(const std::vector<float>& x, float t, std::vector<float>* dxdt) const {
            //initialize derivatives at zero
            if(x.size() != dxdt->size()){
                throw "Size of x and dxdt are not the same";
            }
            if(dxdt->size()%2!=0){
                throw "x should be multiple of 2: x,v";
            }
            //v is both the size of positions and velocity as well as
            // the starting point vor velocities
            int v = dxdt->size()/2;
            
            
            std::vector<float> acc(v);
            //initialize dxdt and
            //initialize dvdt
            for(int i = 0; i<v; i++){
                dxdt->at(i)=x.at(i+v);
                acc.at(i) = 0;
            }
            
            
            //accumulate dvdt in acc
            for(int i = 0; i<forces.size(); i++){
                forces.at(i)->dvdtAcc(x,t,masses,&acc);
            }
            
            //copy acc to the corresponding part of dxdt
            for(int i = v; i<dxdt->size(); i++){
                dxdt->at(i) = acc.at(i-v);
            }
            
            for(auto it: fixed){
                dxdt->at(it*3) = 0;
                dxdt->at(it*3+1) = 0;
                dxdt->at(it*3+2) = 0;
                dxdt->at(v+it*3) = 0;
                dxdt->at(v+it*3+1) = 0;
                dxdt->at(v+it*3+2) = 0;
            }
            return;
        }
};
#endif