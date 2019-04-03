#ifndef UTIL_H
#define UTIL_H

namespace math {

  class vec2 {
  public:
    vec2() = delete;
    vec2(float x, float y): mX(x), mY(y) {}
    vec2(float point): mX(point), mY(point) {}
    vec2(const vec2& point): mX(point.GetX()), mY(point.GetY()) {}

    inline float GetX() const { return mX;}
    inline float GetY() const { return mY;}

    inline void SetX(const float x) { mX = x;}
    inline void SetY(const float y) { mY = y;}

  private:
    float mX;
    float mY;
  };

  class vec3 {
  public:
    vec3() = delete;
    vec3(float x, float y, float z): mX(x), mY(y), mZ(z) {}
    vec3(float point): mX(point), mY(point), mZ(point) {}
    vec3(const vec3& point): mX(point.GetX()), mY(point.GetY()), mZ(point.GetZ()){};

    inline float GetX() const { return mX;}
    inline float GetY() const { return mY;}
    inline float GetZ() const { return mZ;}

  private:
    float mX;
    float mY;
    float mZ;
  };

};

#endif //UTIL_H
