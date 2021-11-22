#ifndef NODE_IMAGE
#define NODE_IMAGE
#include "math.h"
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>

class Image{
private:
    RGBColor* pixels;
    int width, height;
public:

    Image(float w, float h): width(w),height(h){
        pixels = new RGBColor[width*height];

    }
    ~Image(){
        delete[] pixels;
    }
    int getWidth() const {
        return width;
    }
    int getHeight() const{
        return height;
    }
    RGBColor get(const Vec2i& coords) const {
        if(coords.x<0 || coords.x>=width || coords.y<0 || coords.y>=height){
            throw("Error: Trying to access pixels outside image");
        }
        return pixels[coords.x+coords.y*width];
    }
    void set(const Vec2i& coords, const RGBColor& c){
        pixels[coords.x+coords.y*width] = c;
    }
    void writePPM(std::string filename){
        std::ofstream f(filename,std::ios::out | std::ios::binary);
        f<<"P6 ";
        f<<std::to_string(width)<<" "<<std::to_string(height);
        f<<" 255 ";
        for(int i = 0; i<height; i++){
            for(int j = 0; j<width; j++){
                RGBColor pix = get(Vec2i(j,i));
                f<<static_cast<unsigned char>(std::max(0,std::min(int(pix.r*256), 255)));
                f<<static_cast<unsigned char>(std::max(0,std::min(int(pix.g*256), 255)));
                f<<static_cast<unsigned char>(std::max(0,std::min(int(pix.b*256), 255)));
            }
        }
        f.close();
    }

};

#endif