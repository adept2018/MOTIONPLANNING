#pragma once

namespace math {

// the purpose of the vec2 is to have units
// thus private members
  class vec2 {
  public:
    constexpr vec2() = delete;
    constexpr vec2(const vec2&) = default;
    constexpr vec2(vec2&&) = default;

    constexpr explicit vec2(float point): x(point), y(point) {}
    constexpr explicit vec2(float ox, float oy): x(ox), y(oy) {}

    // constexpr vec2(const vec2& point): mX(point.GetX()), mY(point.GetY()) {}

    // constexpr vec2& operator=(const vec2&) = default;
    // constexpr vec2& operator=(vec2&&) = default;

  public:
    float x;
    float y;
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
