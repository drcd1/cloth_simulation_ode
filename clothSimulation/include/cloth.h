#ifndef NODE_CLOTH
#define NODE_CLOTH
#include "ode.h"
#include "solvers.h"
#include "../../renderer/include/primitive.h"
class Cloth{
    private:
        int n;
        int m;
        float l;
        float s;
        std::vector<int> fixed;
        float dmass;
        float k;
        float stiffness;
        float damping;
        ODE_func* f;

     
        int getIdx(int i, int j) const {
            return i*(m+1)+j;
        }
        std::pair<int,int> getIdxs(int idx) const {
            return {int(idx/(m+1)),idx%(m+1)};
        }

        Mesh* createMesh(std::vector<float> x) const {
            Mesh* mesh = new Mesh();
            if(x.size()%6 != 0 || x.size() != 6*(n+1)*(m+1)){
                throw "Unexpected x size when creating mesh";
            }
            for(int i = 0; i< ((n+1)*(m+1)); i++){
                mesh->addVertex(Vec3(x.at(3*i),x.at(3*i+1),x.at(3*i+2)));
            }
            for(int i = 0; i<n; i++){
                for(int j = 0; j<m; j++){
                    mesh->addFace({getIdx(i,j),getIdx(i+1,j), getIdx(i,j+1)});
                    mesh->addFace({getIdx(i+1,j+1), getIdx(i,j+1),getIdx(i+1,j)});
                }
            }
            mesh->updateFaceNormals();
            mesh->updateVertexNormals();
            return mesh;
        }
    public:
        Cloth(int n, int m, float s, std::vector<std::pair<int,int>> a_fixed, float mass, float elasticity, float stiffness,float damping,  std::vector<Force*> external_forces ):
            n(n),m(m),l(s/n), s(s), dmass(mass/((n+1)*(m+1))),k(1.0f/elasticity), stiffness(stiffness),damping(damping)
        {
            for(auto p: a_fixed){
                fixed.push_back(getIdx(p.first,p.second));
            }

            std::vector<Force*> forces;
            for(auto f: external_forces){
                forces.push_back(f);
            }
            
            float lsq = sqrt(2*l*l);

            if(damping<(0.00001)){
            
                //vertical springs
                for(int i = 0; i<n+1; i++){
                    for(int j = 0; j<m; j++){
                        forces.push_back(new SpringForce(getIdx(i,j),getIdx(i,j+1),k,l));
                    }
                }

                //horizontal springs
                for(int i = 0; i<n; i++){
                    for(int j = 0; j<m+1; j++){
                        forces.push_back(new SpringForce(getIdx(i,j),getIdx(i+1,j),k,l));
                    }
                }

                //diagonal springs
                for(int i = 0; i<n; i++){
                    for(int j = 0; j<m; j++){
                        forces.push_back(new SpringForce(getIdx(i,j),getIdx(i+1,j+1),k,lsq));
                        forces.push_back(new SpringForce(getIdx(i+1,j),getIdx(i,j+1),k,lsq));
                    }
                }

                //vertical stiffness
                 for(int i = 0; i<n+1; i++){
                    for(int j = 0; j<m-1; j++){
                        forces.push_back(new SpringForce(getIdx(i,j),getIdx(i,j+2),stiffness,2*l));
                    }
                }

                //horizontal stiffness
                for(int i = 0; i<n-1; i++){
                    for(int j = 0; j<m+1; j++){
                        forces.push_back(new SpringForce(getIdx(i,j),getIdx(i+2,j),stiffness,2*l));
                    }
                }
            } else {
                //vertical springs
                for(int i = 0; i<n+1; i++){
                    for(int j = 0; j<m; j++){
                        forces.push_back(new DampedSpringForce(getIdx(i,j),getIdx(i,j+1),k,l,damping));
                    }
                }

                //horizontal springs
                for(int i = 0; i<n; i++){
                    for(int j = 0; j<m+1; j++){
                        forces.push_back(new DampedSpringForce(getIdx(i,j),getIdx(i+1,j),k,l,damping));
                    }
                }

                //diagonal springs
                for(int i = 0; i<n; i++){
                    for(int j = 0; j<m; j++){
                        forces.push_back(new DampedSpringForce(getIdx(i,j),getIdx(i+1,j+1),k,lsq,damping));
                        forces.push_back(new DampedSpringForce(getIdx(i+1,j),getIdx(i,j+1),k,lsq,damping));
                    }
                }

                //vertical stiffness
                 for(int i = 0; i<n+1; i++){
                    for(int j = 0; j<m-1; j++){
                        forces.push_back(new DampedSpringForce(getIdx(i,j),getIdx(i,j+2),stiffness,2*l,damping));
                    }
                }

                //horizontal stiffness
                for(int i = 0; i<n-1; i++){
                    for(int j = 0; j<m+1; j++){
                        forces.push_back(new DampedSpringForce(getIdx(i,j),getIdx(i+2,j),stiffness,2*l,damping));
                    }
                }
            }
            std::vector<float> masses((n+1)*(m+1));
            for(int i =0; i<masses.size(); i++){
                masses.at(i) = dmass;
            }

            f = new Physics_func(forces,fixed,masses);

        }

        //for rendering only
        std::string clothData() const {
            std::stringstream data;
            data<<n<<" "<<m;
            return data.str();
        }

        std::vector<float> x_ini(){
            std::vector<float> x;
            for(int i = 0; i<(n+1)*(m+1); i++){
                auto idx = getIdxs(i);
                float xPos = (idx.first-(float(n+1)*0.5f))*l;                
                float yPos = (idx.second-(float(m+1)*0.5f))*l;
                x.push_back(xPos);
                x.push_back(yPos);
                x.push_back(0);
            }

            for(int i = 0; i<(n+1)*(m+1); i++){
                x.push_back(0);
                x.push_back(0);
                x.push_back(0);
            }
            return x;
        }
        
        void fillMesh(std::vector<float> x, Mesh** mesh) const {
            if(*mesh==nullptr){
                *mesh = createMesh(x);
            } else {
                for(int i = 0; i<(*mesh)->vertices.size(); i++){
                    (*mesh)->vertices.at(i) = Vec3(x.at(3*i),x.at(3*i+1),x.at(3*i+2));
                }
                (*mesh)->updateFaceNormals();
                (*mesh)->updateVertexNormals();
            }
        }

        ~Cloth(){
            delete f;
        }
        std::vector<std::array<int,3>> getTriangles(){
            std::vector<std::array<int,3>> triangles;            
            for(int i = 0; i<n; i++){
                for(int j = 0; j<m; j++){
                    triangles.push_back({getIdx(i,j),getIdx(i+1,j), getIdx(i,j+1)});
                    triangles.push_back({getIdx(i+1,j+1), getIdx(i,j+1),getIdx(i+1,j)});
                }
            } 
            
            return triangles;
        }

        ODE_func* getFunc(){
            return f;
        }
};

#endif