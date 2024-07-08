from io import TextIOWrapper
import numpy
import scipy.io as sio
from argparse import ArgumentParser

def write_array(file:TextIOWrapper, array:numpy.ndarray):
    for i in range(array.shape[0]):
        for j in range(array.shape[1]):
            file.write(str(array[i][j]) if not numpy.isnan(array[i][j]) else "0.0")
            file.write("\n")


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", help="Input file", required=True)
    parser.add_argument("-o", "--output", help="Output file", required=True)
    args = parser.parse_args()
    mat = sio.loadmat(args.input)
    with open(args.output, "w") as f:
        f.write(str(mat['imu_gyr'].shape[0]))
        f.write("\n")
    
        print(f"Writing gyr: {mat['imu_gyr'].shape}")
        write_array(f, mat['imu_gyr'])
        print(f"Writing acc: {mat['imu_acc'].shape}")
        write_array(f, mat['imu_acc'])
        print(f"Writing mag: {mat['imu_mag'].shape}")
        write_array(f, mat['imu_mag'])
        print(f"Writing quat: {mat['opt_quat'].shape}")
        write_array(f, mat['opt_quat'])
        print(f"Writing pos: {mat['opt_pos'].shape}")
        write_array(f, mat['opt_pos'])
        print(f"Writing movement: {mat['movement'].shape}")
        write_array(f, mat['movement'])


