#include <cmath>
#include <iostream>

template <typename T>
class Vector3 {
public:
  // Constructors
  Vector3() : x(0), y(0), z(0) {}
  Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

  // Comparison operators
  bool operator==(const Vector3<T>& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator!=(const Vector3<T>& other) const {
    return !(*this == other);
  }

  // Arithmetic operators
  Vector3<T> operator+(const Vector3<T>& other) const {
    return Vector3<T>(x + other.x, y + other.y, z + other.z);
  }
  Vector3<T> operator-(const Vector3<T>& other) const {
    return Vector3<T>(x - other.x, y - other.y, z - other.z);
  }
  Vector3<T> operator*(T scalar) const {
    return Vector3<T>(x * scalar, y * scalar, z * scalar);
  }
  Vector3<T> operator/(T scalar) const {
    if (scalar == 0) {
      throw std::invalid_argument("Division by zero");
    }
    return Vector3<T>(x / scalar, y / scalar, z / scalar);
  }
  Vector3<T> operator-() const {
    return Vector3<T>(-x, -y, -z);
  }

  // Assignment operators
  Vector3<T>& operator+=(const Vector3<T>& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  Vector3<T>& operator-=(const Vector3<T>& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
  Vector3<T>& operator*=(T scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }
  Vector3<T>& operator/=(T scalar) {
    if (scalar == 0) {
      throw std::invalid_argument("Division by zero");
    }
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  // Dot product
  T dot(const Vector3<T>& other) const {
    return x * other.x + y * other.y + z * other.z;
  }

  // Cross product
  Vector3<T> cross(const Vector3<T>& other) const {
    return Vector3<T>(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  }

  // Magnitude
  T magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
  }

  // Normalization
  Vector3<T> normalized() const {
    T mag = magnitude();
    if (mag == 0) {
      throw std::invalid_argument("Cannot normalize zero vector");
    }
    return *this / mag;
  }

  // String representation
  std::string toString() const {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
  }

  // Accessors
  T getX() const { return x; }
  T getY() const { return y; }
  T getZ() const { return z; }

private:
  T x, y, z;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& v) {
  os << v.toString();
  return os;
}

// int main(int argc, char const *argv[])
// {
//     Vector3<double> v1(1,2,3);
//     Vector3<double> v2(4,5,6);
//     Vector3<double> v3 = v1.cross(v2);
//     std::cout << v3 << std::endl;
//     std::cout << Vector3<double>(1,2,0).cross(Vector3<double>(4,5,6)) << std::endl;
//     return 0;
// }
