#include <iostream>
#include "../../renderer/include/image.h"
#include <sstream>
#include <iomanip>

#include "../include/cloth.h"
#include "../../renderer/include/renderer.h"
#include <fstream>
//#define PRINT_ENERGY
#define RES 512
void makeScene1(Camera** c, Cloth** cloth){
    *c = new Camera (Vec3(0.5,1,2),Vec3(0,0,0),Vec3(0,0,1),3.1415/3,Vec2i(RES,RES));
    std::vector<Force*> external_forces;
    external_forces.push_back(new GravityForce(Vec3(0,0,-9.8)));
    *cloth = new Cloth(30,30,2,{{0,0},{0,30},{30,0}},0.2f,0.2f,1.0f,0.0f,external_forces);
}

void makeScene2(Camera** c, Cloth** cloth){
    *c = new Camera (Vec3(1,4,0.2),Vec3(-1,0,-1),Vec3(0,0,1),3.1415/3,Vec2i(RES,RES));
    std::vector<Force*> external_forces;
    external_forces.push_back(new GravityForce(Vec3(0,0,-9.8)));
    *cloth = new Cloth(30,30,2,{{0,0},{0,30}},0.2f,0.05f,1.0f,0.0f,external_forces);
}
void makeScene3(Camera** c, Cloth** cloth){
    *c = new Camera (Vec3(1,4,0.2),Vec3(-1,0,-1),Vec3(0,0,1),3.1415/3,Vec2i(RES,RES));
    std::vector<Force*> external_forces;
    external_forces.push_back(new GravityForce(Vec3(0,0,-9.8)));
    Cloth tmp(30,30,1,{},1,1,1,1,{});  
    external_forces.push_back(new AirResistance(tmp.getTriangles(),50.0f));
    *cloth = new Cloth(30,30,2,{{0,0},{0,30}},0.2f,0.05f,1.0f,0.0005f,external_forces);
}
void makeScene4(Camera** c, Cloth** cloth){
    *c = new Camera (Vec3(3,4,0.2),Vec3(1,0,-1),Vec3(0,0,1),3.1415/3,Vec2i(RES,RES));
    std::vector<Force*> external_forces;
    external_forces.push_back(new GravityForce(Vec3(0,0,-9.8)));
    Cloth tmp(30,30,1,{},1,1,1,1,{});  
    external_forces.push_back(new AirResistance(tmp.getTriangles(),50.0f));
    external_forces.push_back(new CapsulePenaltyForceDamped(Vec3(0,0,-1), Vec3(-1,0,-0.5),0.4,0.05,1000,1000));
    external_forces.push_back(new CapsulePenaltyForceDamped(Vec3(0,0,-2), Vec3(2,0,-2),0.4,0.05,1000,1000));
    *cloth = new Cloth(30,30,2,{},0.2f,0.05f,1.0f,0.0005f,external_forces);
}
void makeScene5(Camera** c, Cloth** cloth){
    *c = new Camera (Vec3(1,4,0.2),Vec3(-1,0,-1),Vec3(0,0,1),3.1415/3,Vec2i(RES,RES));
    std::vector<Force*> external_forces;
    external_forces.push_back(new GravityForce(Vec3(0,0,-9.8)));
    Cloth tmp(30,30,1,{},1,1,1,1,{});  
    external_forces.push_back(new AirResistance(tmp.getTriangles(),50.0f));
    external_forces.push_back(new CapsulePenaltyForceDamped(Vec3(0.2,0.4,-1), Vec3(0,-0.4,-1),0.2,0.05,1000,1000));
    *cloth = new Cloth(30,30,2,{{0,0},{0,30}},0.2f,0.05f,1.0f,0.001f,external_forces);
}
void makeScene6(Camera** c, Cloth** cloth){
    *c = new Camera (Vec3(0.5,1,2),Vec3(0,0,0),Vec3(0,0,1),3.1415/3,Vec2i(RES,RES));
    std::vector<Force*> external_forces;
    external_forces.push_back(new GravityForce(Vec3(0,0,-9.8)));
    Cloth tmp(30,30,1,{},1,1,1,1,{});  
    external_forces.push_back(new AirResistance(tmp.getTriangles(),50.0f));
    *cloth = new Cloth(30,30,2,{{0,0},{0,30},{30,0}},0.2f,0.2f,1.0f,0.002f,external_forces);
}

void makeScene7(Camera** c, Cloth** cloth){
    *c = new Camera (Vec3(2,3,0.2),Vec3(0,0,-1),Vec3(0,0,1),3.1415/3,Vec2i(RES,RES));
    std::vector<Force*> external_forces;
    external_forces.push_back(new GravityForce(Vec3(0,0,-9.8)));
    Cloth tmp(30,30,1,{},1,1,1,1,{});  
    external_forces.push_back(new AirResistance(tmp.getTriangles(),50.0f));
    external_forces.push_back(new CapsulePenaltyForceDamped(Vec3(0,0,-1), Vec3(0.01,0,-1.0),0.4,0.05,1000,0.0f));
    *cloth = new Cloth(30,30,2,{},0.2f,0.05f,1.0f,0.0005f,external_forces);   
}


void writeDataFile(std::string filename, const std::vector<std::vector<float>>& traj, const Camera* c, const Cloth* cloth){
    std::fstream os(filename, std::ios::out);
    os<<c->cameraData()<<" "<<cloth->clothData()<<std::endl;
    os<<traj.size()<<" "<<traj.at(0).size()<<std::endl;
    for(auto v: traj){
        for(auto value: v){
            os<<value<<" ";
        }
        os<<std::endl;
    }
    os.close();
}

std::vector<std::vector<float>> readDataFile(std::string filename, Camera** c, Cloth** cloth){
    
    std::fstream is(filename, std::ios::in);
    Vec3 o, at,up;
    float fovy;
    int resx,resy;
    int n,m;
    int x,y;
    is>>o.x>>o.y>>o.z>>at.x>>at.y>>at.z>>up.x>>up.y>>up.z>>fovy>>resx>>resy>>n>>m>>x>>y;
    *cloth = new Cloth(n,m,1,{},1,1,1,1,{});
    *c = new Camera(o,at,up,fovy, Vec2i(resx,resy));
    std::vector<std::vector<float>> traj;
    
    float a;
    for(int i = 0; i<x;i++){
        traj.push_back(std::vector<float>());
        for(int j = 0; j<y; j++){
            is>>a;
            traj.at(i).push_back(a);
        }
    }
    is.close();
    return traj;

}

int main(){
    std::cout<<"Welcome to the cloth simulation program!"<<std::endl;
    std::cout<<""<<std::endl;
    std::cout<<"Please choose one of the following options (insert an integer and press enter):"<<std::endl;
    std::cout<<"1. Simulate scene and render to image sequence"<<std::endl;
    std::cout<<"2. Simulate scene and save to data file"<<std::endl;
    std::cout<<"3. Read simulation from data file and render"<<std::endl;
    int choice=0;
    std::cin>>choice;
    while(!(choice<=3 && choice>=1)){
        std::cout<<"Please insert a valid number."<<std::endl;
        std::cin>>choice;
    }
    Camera* camera = nullptr;
    Cloth* cloth = nullptr;

    std::cout<<"You chose: "<<choice<<std::endl<<std::endl;
    ODESolver* solver = nullptr;
    std::vector<std::vector<float>> traj;
    if(choice ==1 || choice == 2){
        std::cout<<"Please choose the scene to simulate:"<<std::endl;
        std::cout<<"1. Cloth pinned by 3 points (no dissipative forces)"<<std::endl;
        std::cout<<"2. Cloth pinned by 2 points (no dissipative forces)"<<std::endl;
        std::cout<<"3. Cloth pinned by 2 points"<<std::endl;
        std::cout<<"4. Cloth free falling with 2 capsules"<<std::endl;
        std::cout<<"5. Cloth pinned by 2 points hitting capsule"<<std::endl;
        std::cout<<"6. Cloth pinned by 3 points"<<std::endl;
        std::cout<<"7. Cloth free falling hitting a sphere"<<std::endl<<std::endl;
        int scene = 0;
        std::cin>>scene;
        switch(scene){
            case 1:
                makeScene1(&camera,&cloth);
                break;
            case 2:
                makeScene2(&camera,&cloth);
                break;
            case 3:
                makeScene3(&camera,&cloth);
                break;
            case 4:
                makeScene4(&camera,&cloth);
                break;
            case 5:
                makeScene5(&camera,&cloth);
                break;
            case 6:
                makeScene6(&camera,&cloth);
                break;
            case 7:
                makeScene7(&camera,&cloth);
                break;
            default:
                std::cout<<"Unknown scene. Exiting program."<<std::endl;
                return 0;
        }

        std::cout<<"Please choose the integrator:"<<std::endl;
        std::cout<<"1. Explicit Euler"<<std::endl;
        std::cout<<"2. Implicit Euler"<<std::endl;
        std::cout<<"3. Symplectic Euler"<<std::endl;
        std::cout<<"4. Stormer-Verlet"<<std::endl;
        std::cout<<"5. RK4 (\"The\" 4th order Runge-kutta)"<<std::endl;
        std::cout<<"6. Adams Method (order 4)"<<std::endl<<std::endl;
        int integrator = 0;
        std::cin>>integrator;
        switch (integrator)
        {
        case 1: 
            std::cout<<"Using explicit euler..."<<std::endl;
            solver = new ExplicitEuler(cloth->getFunc());
            break;
        case 2: 
            std::cout<<"Using implicit euler..."<<std::endl;
            solver = new ImplicitEuler(cloth->getFunc());
            break;
        case 3: 
            std::cout<<"Using symplectic euler..."<<std::endl;
            solver = new SymplecticEuler(cloth->getFunc());
            break;
        case 4: 
            std::cout<<"Using the Stormer-Verlet scheme..."<<std::endl;
            solver = new StormerVerlet(cloth->getFunc());
            break;
        case 5: 
            std::cout<<"Using  RK4 (\"The\" 4th order Runge-kutta)"<<std::endl;
            solver = new RungeKutta4(cloth->getFunc());
            break;
        case 6: 
            std::cout<<"Adams Method (order 4)"<<std::endl;
            solver = new Adams4(cloth->getFunc());
            break;
        default:
            std::cout<<"Unknown integrator. Exiting program."<<std::endl;
            return 0;
            break;
        }
        
        int frames, subframes;
        float t;
        std::cout<<"Please choose the number of frames to render, time between each frame and the number of substeps (input one integer, one float and one integer):"<<std::endl;
        std::cin>>frames;
        std::cin>>t;
        std::cin>>subframes;

        std::cout<<"Solving..."<<std::endl;
        traj = solver->solve(cloth->x_ini(),0,frames,subframes,t);

        #ifdef PRINT_ENERGY
        std::cout<<"Kinectic Energy     Potential Energy"<<std::endl;
        float time = 0;
        for(auto x: traj){
            float u = 0;
            float k = 0;
            solver->f->potentialEnergy(x,t,&u);
            solver->f->kinecticEnergy(x,t,&k);
            std::cout<<k<<" "<<u<<" "<<u+k<<std::endl;
            time+=t;
        }

        #endif

    }

    

    if(choice==2){
        std::string filename;
        std::cout<<"Please enter the prefix of the data file to save the solution in text format (input a string)"<<std::endl;
        std::cin>>filename;
        writeDataFile(filename+ ".dat",traj, camera, cloth);
        return 0;
    }

    if(choice==3){
        std::string filename;
        std::cout<<"Please enter the name of data file to read the solution in text format (input a string)"<<std::endl;
        std::cin>>filename;
        traj = readDataFile(filename,&camera,&cloth);
    }


    
    
    
    
    

   
    try{
    std::string animName;
    std::cout<<"Please enter the name of the prefix of the animation image files(input a string)"<<std::endl;
    std::cin>>animName;  

    Renderer renderer;
    Scene sc;

    Mesh* m = nullptr;
    cloth->fillMesh(traj.at(0),&m);
    sc.addMesh(m);
    
    std::cout<<"starting render cycle..."<<std::endl;
    for(int t = 0; t<traj.size(); t++){
        std::cout<<"loading data..."<<std::endl;
        cloth->fillMesh(traj.at(t),&m);
        sc.updateGeometry();
        std::cout<<"rendering..."<<std::endl;
        renderer.render(sc,camera);        
        std::stringstream ss;
        ss<<animName<<std::setw(log(traj.size()+1)/log(10) +1)<<std::setfill('0')<<t<<".ppm";
        std::cout<<"Writing: "<<ss.str()<<std::endl;
        camera->im().writePPM(ss.str());
    }
    } catch(const char* c){
        std::cout<<"Exception: "<<c<<std::endl;
        return 0;
    }
    if(solver)
        delete solver;
    if(camera)
        delete camera;
    return 0;
}