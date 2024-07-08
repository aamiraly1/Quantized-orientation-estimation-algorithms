#include <cmath>
#include <iostream>
template <typename T>
struct Quaternion
{
    T w;
    T x;
    T y;
    T z;

    Quaternion(T w, T x, T y, T z) : w(w), x(x), y(y), z(z) {
        //normalize();
    }

    T norm() const {
        return sqrt(w * w + x * x + y * y + z * z);
    }

    // Conjugate
    Quaternion<T> operator-() const {
        return Quaternion<T>(w, -x, -y, -z);
    }

    // Method for quaternion multiplication
    Quaternion<T> operator*(const Quaternion<T> &other) const
    {
        T w1 = w, x1 = x, y1 = y, z1 = z;
        T w2 = other.w, x2 = other.x, y2 = other.y, z2 = other.z;

        T w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        T x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        T y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        T z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        // result is normalized
        Quaternion<T> q(w, x, y, z);
        q.normalize();
        return q;
    }

    // Inverse
    Quaternion<T> operator~() const
    {
        T _norm = norm();
        T epsilon = T(1e-10);
        // Handle potential division by zero
        if (std::abs(_norm) <= epsilon)
        {
            return Quaternion(1.0, 0.0, 0.0, 0.0);
        }
        return (-(*this))/_norm;
        // return Quaternion<T>(w / norm, -x / norm, -y / norm, -z / norm);
    }

    // Scalar division
    Quaternion<T> operator/(const T scalar) const {
        return Quaternion<T>(w/scalar, x/scalar, y/scalar, z/scalar);
    }

    void normalize()
    {
        T norm = std::sqrt(w * w + x * x + y * y + z * z);
        T epsilon = T(1e-10); // Tolerance for division by zero

        // Handle potential division by zero
        if (std::abs((double)norm) < epsilon)
        {
            std::cout << "Quaternion is close to zero. Skipping normalization." << std::endl;
            return; // Do nothing if close to zero
        }

        // Divide each component by the norm
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const Quaternion<T>& q) {
    os << "(";
    os << q.w << ", ";
    os << q.x << ", ";
    os << q.y << ", ";
    os << q.z << ")";
    os << std::endl;
    return os;
}
