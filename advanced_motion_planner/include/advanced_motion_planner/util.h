#ifndef UTIL_H
#define UTIL_H

namespace Math {
  struct vec2 {

    vec2() = delete;
    vec2(float x, float y): x(x), y(y) {};
    vec2(float point): x(point), y(point) {};
    vec2(Math::vec2 point): x(point.x), y(point.y) {};

    float x;
    float y;
  };

  struct vec3 {

    vec3() = delete;
    vec2(float x, float y, float z): x(x), y(y), z(z){};
    vec2(float point): x(point), y(point), z(point){};
    vec2(Math::vec3& point): x(point.x), y(point.y), z(point.z){};

    float x;
    float y;
    float z;
  };
};

#endif //UTIL_H
