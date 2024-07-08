#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

#ifndef IMU_DATASET_H
#define IMU_DATASET_H
using namespace std;

class IMUDataset
{
public:
    int row_count;
    double **gyr;
    double **acc;
    double **mag;
    double **quat;
    double **pos;
    int *movement;
    string filename;

    IMUDataset(string filename)
    {
        
        this->filename = filename; 
        ifstream file(filename);
        file >> row_count; 
        read(file, gyr, 3);
        read(file, acc, 3);
        read(file, mag, 3);
        read(file, quat, 4);
        read(file, pos, 3);
        readbool(file, movement); 
        file.close();
    }
    ~IMUDataset()
    {
        for (int i = 0; i < row_count; i++)
        {
            delete[] gyr[i];
            delete[] acc[i];
            delete[] mag[i];
            delete[] quat[i];
            delete[] pos[i];
        }
        delete[] gyr;
        delete[] acc;
        delete[] mag;
        delete[] quat;
        delete[] pos;
        delete[] movement;
    }

private:
    void readbool(ifstream &file, int *&array) {
        // int total = 0;
        array = new int[row_count];
        for (int i = 0; i < row_count; i++)
        {
            file >> array[i];
        }
    }

    void read(ifstream &file, double **&array, int col_count)
    {
        array = new double *[row_count];
        for (int i = 0; i < row_count; i++)
        {
            array[i] = new double[col_count];
            for (int j = 0; j < col_count; j++)
            {
                file >> array[i][j]; // reads data from the input stream file
                if (file.fail()) {
                    array[i][j] = nan(""); // Set value to NaN
                    file.clear(); // Clear error, if this is not done, rest of the file can't be read
                    file.ignore(3); // ignore nan characters
                }
            }
        }
    }
};
#endif
