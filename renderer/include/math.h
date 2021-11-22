#include <cmath>
#include <limits>

#ifndef NODE_MATH
#define NODE_MATH

#define EPS 0.001
const float inf = std::numeric_limits<float>::infinity();



class RGBColor {
public:
    float r,g,b;
    RGBColor():r(0),g(0),b(0){}
    RGBColor(float r, float g, float b):r(r),g(g),b(b){}
    RGBColor operator+(const RGBColor& c) const {
        return RGBColor(r+c.r,g+c.g,b+c.b);
    }
    RGBColor operator-(const RGBColor& c) const {
        return RGBColor(r-c.r,g-c.g,b-c.b);
    }
    RGBColor operator*(const RGBColor& c) const{
        return RGBColor(r*c.r,g*c.g,b*c.b);
    }
    RGBColor operator*(float f) const {
        return RGBColor(r*f,g*f,b*f);
    }
};
class Vec3 {
public:
    float x,y,z;
    Vec3():x(0),y(0),z(0){

    }
    Vec3(float x, float y, float z):x(x),y(y),z(z){

    }

    Vec3 operator+(const Vec3& v) const {
        return Vec3(x+v.x,y+v.y,z+v.z);
    }
    Vec3 operator-(const Vec3& v) const {
        return Vec3(x-v.x,y-v.y,z-v.z);
    }
    Vec3 operator-() const {
        return Vec3(-x,-y,-z);
    }
    
    float dot(const Vec3& v) const {
        return x*v.x+y*v.y+z*v.z;
    }
    Vec3 cross(const Vec3& v) const {
         return Vec3(y * v.z - z * v.y,
                     z * v.x - x * v.z,
                     x * v.y - y * v.x
                    );
    }

    float lenSqr() const {
        return this->dot(*this);
    }

    float len() const {
        return sqrt(this->lenSqr());
    }
    Vec3 operator*(const Vec3& v) const{
        return Vec3(x*v.x,y*v.y,z*v.z);
    }
    Vec3 operator*(float f) const {
        return Vec3(x*f,y*f,z*f);
    }
    Vec3 operator/(float f) const{
        float factor = 1.0f/f;
        return (*this)*factor;
    }

    Vec3 normalized() const {
        float factor = 1.0f/(this->len());
        return (*this)*factor;
    }

    float operator[](int i) const {
        switch(i){
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                throw "Unknown index for vec3";
        }
    }


};
class Vec2{
    public:
        float x,y;
        Vec2(): x(0),y(0){}
        Vec2(float x,float y):x(x),y(y){}
        Vec2 operator+(const Vec2& v) const {
            return Vec2(x+v.x,y+v.y);
        }
        Vec2 operator-(const Vec2& v) const {
            return Vec2(x-v.x,y-v.y);
        }
        Vec2 operator*(const Vec2& v) const {
            return Vec2(x*v.x, y*v.y);
        }
};
class Vec2i{
    public:
        int x,y;
        Vec2i(): x(0),y(0){}
        Vec2i(int x,int y):x(x),y(y){}
};

class Mat4 {
private:
    float m[16];
public:
    Mat4(){
        m[0] = 1;
        m[1] = 0;
        m[2] = 0;
        m[3] = 0;

        m[4] = 0;
        m[5] = 1;
        m[6] = 0;
        m[7] = 0;

        m[8] = 0;
        m[9] = 0;
        m[10]= 1;
        m[11]= 0;

        m[12]= 0;
        m[13]= 0;
        m[14]= 0;
        m[15]= 1;
        
    }
    Mat4(float m00, float m01, float m02, float m03,
         float m10, float m11, float m12, float m13,
         float m20, float m21, float m22, float m23,
         float m30, float m31, float m32, float m33){
        
        m[0] = m00;
        m[1] = m01;
        m[2] = m02;
        m[3] = m03;

        m[4] = m10;
        m[5] = m11;
        m[6] = m12;
        m[7] = m13;

        m[8] = m20;
        m[9] = m21;
        m[10] = m22;
        m[11] = m23;

        m[12] = m30;
        m[13] = m31;
        m[14] = m32;
        m[15] = m33;
    }

    Vec3 multiplyByVec3(Vec3& v){
        return Vec3(
            m[0]*v.x+m[1]*v.y+m[2]*v.z,
            m[4]*v.x+m[5]*v.y+m[6]*v.z,
            m[8]*v.x+m[9]*v.y+m[10]*v.z
        );
    }
    Vec3 multiplyByPoint3(Vec3& v){
        return Vec3(
            m[0]*v.x+m[1]*v.y+m[2]*v.z + m[3],
            m[4]*v.x+m[5]*v.y+m[6]*v.z + m[7],
            m[8]*v.x+m[9]*v.y+m[10]*v.z + m[11]
        );
    }

};

class Ray{
    public:
        Ray(const Vec3& o,const Vec3& d,float t = inf): o(o),d(d),maxT(t){}
        Vec3 o, d; 
        float maxT;
};
#endif