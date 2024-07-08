#include <iostream>
#include <string>
#include <fstream>

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
    double gyrMin;
    double gyrMax;
    double accMin;
    double accMax;
    double magMin;
    double magMax;
    double quatMin;
    double quatMax;
    double posMin;
    double posMax;
    string filename;

    IMUDataset(string filename)
    {

        this->filename = filename;
        gyrMin = accMin = magMin = quatMin = posMin = 9999999.0;
        gyrMax = accMax = magMax = quatMax = posMax = -9999999.0;
        ifstream file(filename);
        file >> row_count;
        read(file, gyr, 3, gyrMin, gyrMax);
        read(file, acc, 3, accMin, accMax);
        read(file, mag, 3, magMin, magMax);
        read(file, quat, 4, quatMin, quatMax);
        read(file, pos, 3, posMin, posMax);
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
    }

    friend ostream &operator<<(ostream &os, const IMUDataset &dataset)
    {
        os << "gyrMin: " << dataset.gyrMin << endl;
        os << "gyrMax: " << dataset.gyrMax << endl;
        os << "accMin: " << dataset.accMin << endl;
        os << "accMax: " << dataset.accMax << endl;
        os << "magMin: " << dataset.magMin << endl;
        os << "magMax: " << dataset.magMax << endl;
        os << "quatMin: " << dataset.quatMin << endl;
        os << "quatMax: " << dataset.quatMax << endl;
        os << "posMin: " << dataset.posMin << endl;
        os << "posMax: " << dataset.posMax << endl;
        os << endl;
        return os;
    }

private:
    void read(ifstream &file, double **&array, int col_count, double &min, double &max)
    {
        array = new double *[row_count];
        for (int i = 0; i < row_count; i++)
        {
            array[i] = new double[col_count];
            for (int j = 0; j < col_count; j++)
            {
                file >> array[i][j];
                if (file.fail())
                    cout << i << ", " << j << " " << array << endl;

                if (array[i][j] < min)
                    min = array[i][j];
                if (array[i][j] > max)
                    max = array[i][j];
            }
        }
    }
};

int main(int argc, char const *argv[])
{
    IMUDataset dataset1("01.dat");
    IMUDataset dataset2("02.dat");
    IMUDataset dataset3("03.dat");
    IMUDataset dataset4("04.dat");
    IMUDataset dataset5("05.dat");
    IMUDataset dataset6("06.dat");
    IMUDataset dataset7("07.dat");
    IMUDataset dataset8("08.dat");
    IMUDataset dataset9("09.dat");
    IMUDataset dataset10("10.dat");
    IMUDataset dataset11("11.dat");
    IMUDataset dataset12("12.dat");
    IMUDataset dataset13("13.dat");
    IMUDataset dataset14("14.dat");
    IMUDataset dataset15("15.dat");
    IMUDataset dataset16("16.dat");
    IMUDataset dataset17("17.dat");
    IMUDataset dataset18("18.dat");
    IMUDataset dataset19("19.dat");
    IMUDataset dataset20("20.dat");
    IMUDataset dataset21("21.dat");
    IMUDataset dataset22("22.dat");
    IMUDataset dataset23("23.dat");
    IMUDataset dataset24("24.dat");
    IMUDataset dataset25("25.dat");
    IMUDataset dataset26("26.dat");
    IMUDataset dataset27("27.dat");
    IMUDataset dataset28("28.dat");
    IMUDataset dataset29("29.dat");
    IMUDataset dataset30("30.dat");
    IMUDataset dataset31("31.dat");
    IMUDataset dataset32("32.dat");
    IMUDataset dataset33("33.dat");
    IMUDataset dataset34("34.dat");
    IMUDataset dataset35("35.dat");
    IMUDataset dataset36("36.dat");
    IMUDataset dataset37("37.dat");
    IMUDataset dataset38("38.dat");
    IMUDataset dataset39("39.dat");
    
    cout << dataset1;
    cout << dataset2;
    cout << dataset3;
    cout << dataset4;
    cout << dataset5;
    cout << dataset6;
    cout << dataset7;
    cout << dataset8;
    cout << dataset9;
    cout << dataset10;
    cout << dataset11;
    cout << dataset12;
    cout << dataset13;
    cout << dataset14;
    cout << dataset15;
    cout << dataset16;
    cout << dataset17;
    cout << dataset18;
    cout << dataset19;
    cout << dataset20;
    cout << dataset21;
    cout << dataset22;
    cout << dataset23;
    cout << dataset24;
    cout << dataset25;
    cout << dataset26;
    cout << dataset27;
    cout << dataset28;
    cout << dataset29;
    cout << dataset30;
    cout << dataset31;
    cout << dataset32;
    cout << dataset33;
    cout << dataset34;
    cout << dataset35;
    cout << dataset36;
    cout << dataset37;
    cout << dataset38;
    cout << dataset39;
    return 0;
}
//os << dataset.filename << ":" << dataset.row_count << " rows" << endl;