#ifndef NODE_RENDERER
#define NODE_RENDERER
#include "primitive.h"
#include "image.h"
class Camera{
    private:
        
        float aspect;
        Image image;
        float tanFOVy;
        Mat4 transform;
        Vec3 origin;
        Vec3 at;
        Vec3 up;
        float fovy;
        Vec2i res;
    public:
        Camera(const Vec3& o,const Vec3& at,const Vec3& up, float fovy, const Vec2i& res):
            aspect(float(res.x)/res.y),image(res.x,res.y),tanFOVy(tan(fovy*0.5)),
            at(at),up(up),fovy(fovy),res(res){
            
            origin = o;
            
            Vec3 z = (o-at).normalized();
            
            Vec3 x = up.cross(z).normalized();
            Vec3 y = z.cross(x).normalized();
            transform = Mat4(x.x,y.x,z.x,o.x,
                             x.y,y.y,z.y,o.y,
                             x.z,y.z,z.z,o.z,
                               0,  0,  0,  1);
                               
        }

        Ray getRay(const Vec2& coords){
            Vec3 d(coords.x*aspect*tanFOVy, coords.y*tanFOVy, -1);
            d = transform.multiplyByVec3(d).normalized();
            return Ray(origin,d);
        }

        Image& im(){
            return image;
        }
        std::string cameraData() const {
            std::stringstream data;
            data<<origin.x<<" "<<origin.y<<" "<<origin.z<<" "<<at.x<<" "<<at.y<<" "<<at.z<<" "<<up.x<<" "<<up.y<<" "<<up.z<<" "<<fovy<<" "<<res.x<<" "<<res.y;
            return data.str();
        }
        
};

class Sphere: public Primitive {
    private:
        Vec3 center;
        float radius;
        static bool solve_quadratic(float a,float b, float c, float* tmin, float* tmax){
            float d = b*b-4.0f*a*c;
            if(d<0.0f)
                return false;
            *tmin = (-b-sqrt(d))/(2.0f*a);
            *tmax = (-b+sqrt(d))/(2.0f*a);
            return true;
        }
    public:
        Sphere(const Vec3& c, float r): Primitive(BBox(Vec3(c.x-r,c.y-r,c.z-r),Vec3(c.x+r,c.y+r,c.z+r))), center(c), radius(r){
            
        }

        bool intersect(Ray* r, Intersection* it) const {
            Vec3 delta = center - r->o;
   	        float c = delta.lenSqr() - radius*radius;
            float b = -2.0f*(r->d.dot(delta));
            float a =r->d.lenSqr();

            float tMin, tMax;

            if (solve_quadratic(a,b,c,&tMin,&tMax)){
                if(tMin>EPS && tMin<r->maxT){
                    r->maxT = tMin;
                    it->normal = (r->d*r->maxT - delta).normalized();
                    it->pos = r->d*r->maxT + r->o;
                    return true;
                } else if(tMax>EPS && tMax<r->maxT){
                	r->maxT = tMax;
                    it->normal =   (r->d*r->maxT - delta).normalized();
                    it->pos = r->d*r->maxT + r->o;
                    return true;
                } else {
                	return false;
                }

            } else {
                return false;
            }    
        }
        
};

class Scene{
    private:
        std::vector<const Mesh*> meshes;
    public:
        Primitive* geometry;

        ~Scene(){
            if(geometry)
                delete geometry;
        }
        Scene(): geometry(nullptr){

        }
        void addMesh(const Mesh* m){ 
            meshes.push_back(m);
        }

        void updateGeometry(){
            std::vector<Primitive*> primitives;
            for(auto m: meshes){
                for(unsigned int i = 0; i<m->faces.size(); i++){
                    primitives.push_back(new Triangle(i,m));                   
                }
            }
            if(geometry)
                delete geometry;
            //geometry = new Sphere(Vec3(0,0,0),1);
           /* Mesh* a = new Mesh();
            a->addVertex(Vec3(-1,-1,0));
            a->addVertex(Vec3(1,-1,0));
            a->addVertex(Vec3(1,1,0));
            a->addFace({0,1,2});
            a->updateFaceNormals();
            a->updateVertexNormals();
            geometry = new Triangle(0,a);*/
            geometry = new BVH(primitives);
            
        }
};

class Renderer{
public:
    void render(const Scene& sc, Camera* c){
        try{
        for(int i = 0; i<c->im().getWidth();i++){
            //std::cout<<"Collumn "<<i<<std::endl;
            for(int j = 0; j<c->im().getHeight();j++){
                //convert pixels to [-1,1]^2
                Vec2 coord(((i+0.5)/c->im().getWidth())*2.0f-1.0f,((j+0.5)/c->im().getHeight())*2.0f-1.0f);
                RGBColor col = sample(sc,c,coord);
                c->im().set(Vec2i(i,c->im().getHeight()-1-j), col);
            }
        }
        } catch(char const* a){
            std::cout<<a<<std::endl;
        }
    }
private:
    RGBColor background(Vec3 d){
        float gradient = d.z*0.5+0.5;
        Vec3 col = Vec3(0.8,0.86,0.93)*gradient;
        return RGBColor(col.x,col.y,col.z);
    }
    RGBColor sample(const Scene& sc, Camera* c, const Vec2& coord){
        Ray r = c->getRay(coord);
        Intersection it;
        if(sc.geometry->intersect(&r,&it)){
            Vec3 col;
            float front_light = fabs(it.normal.dot(r.d));
            Vec3 main_light_pos(20,3,5);
            float main_light = it.normal.dot((main_light_pos-it.pos).normalized());
            main_light = main_light<0?0:main_light;
            col = Vec3(0.2,0.2,0.2)*front_light + Vec3(0.6,0.6,0.6)*main_light;
            return RGBColor(col.x,col.y,col.z);
        } else {
            return background(r.d);
        }

    }

};
#endif