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
      constexpr vec3() = delete;
      constexpr vec3(const vec3&) = default;
      constexpr vec3(vec3&&) = default;

      constexpr explicit vec3(float point): x(point), y(point), z(point) {}
      constexpr explicit vec3(float ox, float oy, float oz): x(ox), y(oy), z(oz) {}
  public:
      float x;
      float y;
      float z;
  };

};
