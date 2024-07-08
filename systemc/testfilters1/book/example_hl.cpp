class HeatingLiquid {
    public:
    static int count;
    static double measurements[10];
    static double truth[10];
};

int HeatingLiquid::count = 10;
double HeatingLiquid::measurements[10] = {50.486, 50.963, 51.597, 52.001, 52.518, 53.05, 53.438, 53.858, 54.465, 55.114};
double HeatingLiquid::truth[10] = {50.505, 50.994, 51.493, 52.001, 52.506, 52.998, 53.521, 54.005, 54.5, 54.997};