#ifndef NODE_PRIMITIVE
#define NODE_PRIMITIVE
#include <vector>
#include "math.h"
#include <iostream>
#include <algorithm>
#include <iostream>
#include <array>
class Mesh{
    public:
        std::vector<std::array<int,3>> faces;
        std::vector<Vec3> vertices;
        std::vector<Vec3> face_normals;
        std::vector<Vec3> vertex_normals;
        std::vector<std::vector<int>> faces_per_vertex;
        void addVertex(Vec3 v){
            vertices.push_back(v);
            faces_per_vertex.push_back(std::vector<int>());
            vertex_normals.push_back(Vec3(0,0,1));
        }
        void addFace(std::array<int,3> idxs){
            faces.push_back({idxs[0],idxs[1],idxs[2]});
            face_normals.push_back(Vec3(0,0,1));
            faces_per_vertex.at(idxs[0]).push_back(faces.size()-1);
            faces_per_vertex.at(idxs[1]).push_back(faces.size()-1);
            faces_per_vertex.at(idxs[2]).push_back(faces.size()-1);
        }
        void updateVertexNormals(){
            for(unsigned int i = 0; i<vertices.size(); i++){
                Vec3 normal(0,0,0);
                for(auto f: faces_per_vertex.at(i)){
                    normal = normal + face_normals.at(f);
                }
                if(faces_per_vertex.at(i).size()>0){
                    vertex_normals.at(i) = normal.normalized();
                }
            }
            return;
        }
        void updateFaceNormals(){;
            for(unsigned int i = 0; i<faces.size(); i++){
                Vec3 v1 = vertices.at(faces.at(i)[0]);
                Vec3 v2 = vertices.at(faces.at(i)[1]);
                Vec3 v3 = vertices.at(faces.at(i)[2]);

                Vec3 e1 = v2-v1;
                Vec3 e2 = v3-v1;
                Vec3 normal = e1.cross(e2).normalized();
                face_normals.at(i) = normal;
            }
            
            return;
        }
};
class Intersection{
    public:
        Vec3 normal;
        Vec3 pos;
};

class BBox{
private:
    void updateTs(float* tmin,float* tmax,float* oldTmin,float* oldTmax) const {
        float temp;
        if(*tmin>*tmax){
            temp = *tmin;
            *tmin = *tmax;
            *tmax = temp;
        }
        if(*tmin>*tmax){
            throw "This should not happen";
        }
        if(*oldTmin<*tmin){
            *oldTmin = *tmin;
        } else if(*oldTmax>*tmax){
            *oldTmax = *tmax;
        }
    }
public:
    static BBox empty(){
        return BBox(Vec3(inf,inf,inf), Vec3(-inf,-inf,-inf));
    }
    Vec3 min, max;
    BBox(Vec3 min, Vec3 max): min(min),max(max){}
    int getBiggestExtent(){
        float e1 = max.x-min.x;
        float e2 = max.y-min.y;
        float e3 = max.z-min.z;
        if(e1>e2){
            if(e1>e3)
                return 0;
            else
                return 2;
        } else if(e2>e3)
            return 1;
        else
            return 2;
    }
    Vec3 getCenter(){
        return (max+min)*0.5;
    }
    
    bool intersectRayRelaxed(const Ray& rt) const {
        float oldTmin = -inf;
        float oldTmax = inf;

        float inv,tmin,tmax;

        inv = 1.0f/rt.d.x;
        tmin = (min.x-rt.o.x)*inv;
        tmax = (max.x-rt.o.x)*inv;
        updateTs(&tmin,&tmax,&oldTmin,&oldTmax);
        //this may be wrong
        if(oldTmin>(oldTmax+EPS)){
            return false;
        }
        
        inv = 1.0f/rt.d.y;
        tmin = (min.y-rt.o.y)*inv;
        tmax = (max.y-rt.o.y)*inv;
        updateTs(&tmin,&tmax,&oldTmin,&oldTmax);
        //this may be wrong
        if(oldTmin>(oldTmax+EPS)){
            return false;
        }
        
        inv = 1.0f/rt.d.z;
        tmin = (min.z-rt.o.z)*inv;
        tmax = (max.z-rt.o.z)*inv;
        updateTs(&tmin,&tmax,&oldTmin,&oldTmax);
        //this may be wrong
        if(oldTmin>(oldTmax+EPS)){
            return false;
        }

        if(tmin>EPS && tmin<rt.maxT+EPS)
            return true;

        //todo: check this
        if(tmin<=EPS && tmax>-EPS)
            return true;
        return false;
    }
    
    BBox extend(const Vec3& point) const {
        Vec3 newMax(std::max(max.x,point.x), std::max(max.y,point.y),std::max(max.z,point.z));
        Vec3 newMin(std::min(min.x,point.x),std::min(min.y,point.y),std::min(min.z,point.z));
        

        return BBox(newMin,newMax);
    }
    BBox extend(const BBox& bbox) const {
        Vec3 newMax(std::max(max.x,bbox.max.x), std::max(max.y,bbox.max.y),std::max(max.z,bbox.max.z));
        Vec3 newMin(std::min(min.x,bbox.min.x),std::min(min.y,bbox.min.y),std::min(min.z,bbox.min.z));
        return BBox(newMin,newMax);
    }
};
class Primitive{
    public:
        BBox bbox;
        Primitive(): bbox(BBox::empty()){}
        Primitive(const BBox& bbox): bbox(bbox){}
        virtual ~Primitive(){}
        virtual bool intersect(Ray* rt, Intersection* it) const =0;
};

class BVH: public Primitive{
    public:
        BVH(std::vector<Primitive*> primitives){
            for(auto p: primitives){
                bbox = bbox.extend(p->bbox);
            }
            root = new BVHNode(primitives,bbox);
        }
        ~BVH(){
            delete root;
        }
        bool intersect(Ray* rt, Intersection* it) const {
            return root->intersect(rt,it);
        }
    private:
        class BVHNode:public Primitive{
            public:
            BVHNode(const std::vector<Primitive*>& a_primitives, const BBox& a_bbox):Primitive(a_bbox){
                if(a_primitives.size()<5){
                    for(auto p : a_primitives){
                        children.push_back(p);
                    }
                    return;
                }
                //else: we should divide them
                //float extent;
                int dim;
                BBox bb(BBox::empty());
                for(auto p: a_primitives){
                    bb = bb.extend(p->bbox.getCenter());
                }


                dim = bb.getBiggestExtent();
                
                //extent = bbox.max[dim] - bbox.min[dim];
                
                //std::cout<<"extent: "<<extent<<std::endl;
                std::vector<float> c;
                float center = bb.getCenter()[dim];
                for(auto p : a_primitives){
                    //std::cout<<p->bbox.min[dim]<<" "<<p->bbox.max[dim]<<std::endl;
                    
                    c.push_back(p->bbox.getCenter()[dim]);
                    //center += p->bbox.getCenter()[dim];
                }
                //center = center/a_primitives.size();
                std::vector<Primitive*> left;
                std::vector<Primitive*> right;
                std::vector<Primitive*> middle;
                for(unsigned int i = 0; i<a_primitives.size(); i++){
                    //std::cout<<center<<" and "<<c.at(i)<<std::endl;
                    if(c.at(i)<center){
                        left.push_back(a_primitives.at(i));
                    } else {
                        right.push_back(a_primitives.at(i));
                    }
                }

                if(left.size()<1 || right.size()<1){
                    throw "Error: Bad Geometry";
                }
                
                BBox bbox_l = BBox::empty();
                BBox bbox_r = BBox::empty();

                for(auto p: left){
                    bbox_l = bbox_l.extend(p->bbox);
                }
                for(auto p: right){
                    bbox_r = bbox_r.extend(p->bbox);
                }
                children.push_back(new BVHNode(left,bbox_l));
                children.push_back(new BVHNode(right,bbox_r));

            }

            bool intersect(Ray* rt,Intersection* it) const {
                if(bbox.intersectRayRelaxed(*rt)==false){
                   return false;
                }
                bool intersected = false;
                for(unsigned int i = 0; i<children.size(); i++){
                    intersected = children.at(i)->intersect(rt,it) || intersected;
                    
                }
                return intersected;
            }


            private:
                std::vector<Primitive*> children;

        };
        BVHNode* root;
};

class Triangle: public Primitive{
    private:
        const Mesh* m;
        int f;
    public:
        Triangle(int f, const Mesh* m): Primitive(BBox::empty()), m(m),f(f){
            int idxs[3];
            idxs[0] = m->faces.at(f)[0];
            idxs[1] = m->faces.at(f)[1];
            idxs[2] = m->faces.at(f)[2];
  
            bbox = bbox.extend(m->vertices.at(idxs[0]));
            bbox = bbox.extend(m->vertices.at(idxs[1]));
            bbox = bbox.extend(m->vertices.at(idxs[2]));
            
        }
        bool intersect(Ray* rt, Intersection* it) const {
            /*if(rt->maxT<inf){
                std::cout<<rt->maxT<<std::endl;
            }*/
            int idxs[3];
            idxs[0] = m->faces.at(f)[0];
            idxs[1] = m->faces.at(f)[1];
            idxs[2] = m->faces.at(f)[2];

            Vec3 v0 = m->vertices.at(idxs[0]);
            Vec3 v1 = m->vertices.at(idxs[1]);
            Vec3 v2 = m->vertices.at(idxs[2]);

            Vec3 e1 = v1-v0;
            Vec3 e2 = v2-v0;

            Vec3 normal = m->face_normals.at(f);
  
            Vec3 d = rt->o - v0;
            float dist = d.dot(normal);
            float coef = -rt->d.dot(normal);

            //TODO: are we sure?
            if(coef < EPS && -coef < EPS){
                return false;
            }
            float t = dist/coef;
            if(t<EPS || t>rt->maxT){
                return false;
            }

            Vec3 x = rt->o+rt->d*t;

            //let M = [e1 e2 normal]
            //x-v0 = M*[a,b,normal]
            //then, by cramer's rule
            //a = det(x,e2,normal)/det(M)
            //b = det(e1,x,normal)/det(M)

            Vec3 dx = x-v0;

            float dM = e1.x*e2.y*normal.z + e2.x*normal.y*e1.z + normal.x*e1.y*e2.z - normal.x*e2.y*e1.z - e1.x*normal.y*e2.z- e2.x*e1.y*normal.z;

            float dA = dx.x*e2.y*normal.z + e2.x*normal.y*dx.z + normal.x*dx.y*e2.z - normal.x*e2.y*dx.z - dx.x*normal.y*e2.z- e2.x*dx.y*normal.z;
            float dB = e1.x*dx.y*normal.z + dx.x*normal.y*e1.z + normal.x*e1.y*dx.z - normal.x*dx.y*e1.z - e1.x*normal.y*dx.z- dx.x*e1.y*normal.z;

            float a = dA/dM;
            float b = dB/dM;

            if(a<0 || b<0 || a+b>1){
                return false;
            }
            
            rt->maxT = t;
            
            it->normal = (m->vertex_normals.at(idxs[0])*(1-a-b) + m->vertex_normals.at(idxs[1])*a + m->vertex_normals.at(idxs[2])*b).normalized();
            
            it->pos = x;
            return true;
        }
};



#endif