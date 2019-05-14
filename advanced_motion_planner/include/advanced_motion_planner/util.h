#pragma once

namespace math {

    template<typename T>
    struct tvec2 {
        tvec2<T>() = delete;
        constexpr tvec2<T>(const tvec2<T>&) = default;
        constexpr tvec2<T>(tvec2<T>&&) = default;

        tvec2<T>& operator=(const tvec2<T>&) = default;
        tvec2<T>& operator=(tvec2<T>&&) = default;

        constexpr explicit tvec2<T>(T point): x(point), y(point) {}
        constexpr explicit tvec2<T>(T ox, T oy): x(ox), y(oy) {}

        T x, y;
    };

    using vec2 = tvec2<float>;
    using dvec2 = tvec2<double>;
    using ivec2 = tvec2<int>;

    template<typename T>
    struct tvec3 {
      tvec3<T>() = delete;
      constexpr tvec3<T>(const tvec3<T>&) = default;
      constexpr tvec3<T>(tvec3<T>&&) = default;

      tvec3<T>& operator=(const tvec3<T>&) = default;
      tvec3<T>& operator=(tvec3<T>&&) = default;

      constexpr explicit tvec3<T>(T point): x(point), y(point), z(point) {}
      constexpr explicit tvec3<T>(T ox, T oy, T oz): x(ox), y(oy), z(oz) {}

      T x, y, z;
    };

    using vec3 = tvec3<float>;

};
