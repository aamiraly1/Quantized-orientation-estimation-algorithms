#ifndef MATRIX_HPP
#define MATRIX_HPP
#include <vector>
using namespace std;
template <typename T>
class Matrix
{
private:
    vector<vector<T>> data;
    int rows;
    int cols;
    int index;

public:
    // Matrix() : rows(1), cols(1), data(rows, std::vector<T>(cols, 0.0)), index(0) {}
    Matrix(int rows, int cols) : rows(rows), cols(cols), data(rows, std::vector<T>(cols, 0.0)), index(0) {}

    Matrix &operator<<(const T &value)
    {
        data[index / cols][index % cols] = value;
        index = (index + 1) % (cols * rows);
        return *this;
    }

    int getRows() const
    {
        return rows;
    }

    int getCols() const
    {
        return cols;
    }

    T getData(int i, int j) const
    {
        return data[i][j];
    }

    Matrix<T> operator+(const Matrix<T> &m2)
    {
        if (rows != m2.getRows() || cols != m2.getCols())
            throw out_of_range("Matrix dimensions does not match for +");
        Matrix<T> result(rows, cols);

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                result << data[i][j] + m2.getData(i, j);
            }
        }

        return result;
    }

    Matrix<T> operator*(const Matrix<T> &m2)
    {
        if (cols != m2.getRows())
            throw std::out_of_range("Matrix dimensions do not match for *");

        int resultCols = m2.getCols(); // Rename to avoid conflict with member variable
        int commonDim = cols;
        Matrix<T> result(rows, resultCols);

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < resultCols; ++j) // Use resultCols instead of cols
            {
                T sum = 0;
                for (int k = 0; k < commonDim; ++k)
                {
                    sum += getData(i, k) * m2.getData(k, j);
                }
                result << sum;
            }
        }
        return result;
    }

    Matrix<T> transpose()
    {
        Matrix<T> result(cols, rows);

        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                result << getData(i, j);
            }
        }

        return result;
    }

    T& operator()(int i, int j)
    {
        return data[i][j];
    }

    const T& operator()(int i, int j) const
    {
        return data[i][j];
    }

    static Matrix<T> identity(int size)
    {
        Matrix<T> identity(size, size);
        for (int i = 0; i < size; ++i)
            identity(i, i) = 1;

        return identity;
    }

    Matrix<T> inverse() const
    {
        if (rows != cols)
            throw std::runtime_error("Matrix must be square for inversion");

        Matrix<T> result(rows, cols);
        Matrix<T> identity = Matrix<T>::identity(rows);

        // Perform LU decomposition (you need to implement this)
        Matrix<T> lu(rows, rows), pivot(rows, rows);
        luDecomposition(lu, pivot);

        // Solve for the inverse using LU decomposition (you need to implement this)
        Matrix<T> inv = solveLU(lu, pivot, identity);

        return inv;
    }

    void luDecomposition(Matrix<T>& LU, Matrix<T>& pivot) const
    {
        int n = rows;

        // Initialize LU matrix and pivot vector
        LU = *this;
        pivot = Matrix<T>::identity(n);

        for (int k = 0; k < n - 1; ++k)
        {
            // Partial pivoting
            int pivotIndex = k;
            T pivotValue = std::abs(LU(k, k));
            for (int i = k + 1; i < n; ++i)
            {
                if (std::abs(LU(i, k)) > pivotValue)
                {
                    pivotIndex = i;
                    pivotValue = std::abs(LU(i, k));
                }
            }

            // Swap rows in LU and pivot
            if (pivotIndex != k)
            {
                std::swap(LU.data[pivotIndex], LU.data[k]);
                std::swap(pivot.data[pivotIndex], pivot.data[k]);
            }

            // Perform elimination
            for (int i = k + 1; i < n; ++i)
            {
                LU(i, k) /= LU(k, k);
                for (int j = k + 1; j < n; ++j)
                {
                    LU(i, j) -= LU(i, k) * LU(k, j);
                }
            }
        }
    }

    Matrix<T> solveLU(const Matrix<T>& LU, const Matrix<T>& pivot, const Matrix<T>& b) const
    {
        int n = rows;
        Matrix<T> x(n, 1);

        // Forward substitution
        for (int i = 0; i < n; ++i)
        {
            // x(i, 0) = b(pivot(i), 0);
            x(i, 0) = b(i, 0);
            for (int j = 0; j < i; ++j)
            {
                x(i, 0) -= LU(i, j) * x(j, 0);
            }
        }

        // Backward substitution
        for (int i = n - 1; i >= 0; --i)
        {
            for (int j = i + 1; j < n; ++j)
            {
                x(i, 0) -= LU(i, j) * x(j, 0);
            }
            x(i, 0) /= LU(i, i);
        }

        return x;
    }

    Matrix(const Matrix<T>& other) : rows(other.rows), cols(other.cols), index(other.index)
    {
        // Perform deep copy of the data
        data = std::vector<std::vector<T>>(other.data);
    }

    Matrix<T>& operator=(const Matrix<T>& other)
    {
        if (this != &other) // Check for self-assignment
        {
            rows = other.rows;
            cols = other.cols;
            index = other.index;

            // Perform deep copy of the data
            data = std::vector<std::vector<T>>(other.data);
        }
        return *this;
    }    

};

template <typename T>
std::ostream &operator<<(std::ostream &os, const Matrix<T> &matrix)
{
    for (int i = 0; i < matrix.getRows(); ++i)
    {
        for (int j = 0; j < matrix.getCols(); ++j)
        {
            os << matrix.getData(i, j) << ' ';
        }
        os << '\n';
    }
    return os;
}
#endif