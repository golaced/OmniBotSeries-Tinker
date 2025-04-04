  #pragma once
#include<stdint.h>
#include<SD/Seeed_SD.h>


/*
USAGE:

    // when use 8bit color.
    Raw8 * img8 = newImage<uint8_t>("path to sd card image.");

    // when use 16bit color.
    Raw16 * img16 = newImage<uint16_t>("path to sd card image.");

    // do some drawing.
    // img8->draw();

    // remember release it
    img8->release();
    img16->release();
 */

extern TFT_eSPI tft;

template<class type>
struct RawImage{
    type * ptr(){
        return (type *)(this + 1);
    }
    type get(int16_t x, int16_t y){
        return this->ptr()[y * width() + x];
    }
    void draw(size_t x = 0, size_t y = 0){
        tft.pushImage(x, y, width(), height(), ptr());
    }
    void release(){
        delete [] this;
    }
    int16_t width(){ return _width; }
    int16_t height(){ return _height; }
private:
    int16_t  _width;
    int16_t  _height;
};

typedef RawImage<uint8_t>  Raw8;
typedef RawImage<uint16_t> Raw16;

template<class type>
RawImage<type> * newImage(const char * path){
    typedef RawImage<type> raw;
    File f = SD.open(path, FILE_READ);
    if (!f){
        return nullptr;
    }
    int32_t size = f.size();
    raw   * mem = (raw *)new uint8_t[size];
    if (mem == nullptr){
        return nullptr;
    }
    f.read(mem, size);
    f.close();
    return mem;
}

template<class type>
void drawImage(const char * path, size_t x = 0, size_t y = 0){
    auto img = newImage<type>(path);
    img->draw(x, y);
    img->release();
}
