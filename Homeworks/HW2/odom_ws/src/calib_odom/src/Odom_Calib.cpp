#include "../include/calib_odom/Odom_Calib.hpp"


//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len*3, 9);
    b.conservativeResize(len*3);
    A.setZero();
    b.setZero();
}


/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom,Eigen::Vector3d scan)
{

    if(now_len < INT_MAX)
    {
        //TODO: 构建超定方程组
        Eigen::MatrixXd A_i(3, 9);
        A_i << Odom(0), Odom(1), Odom(2), 0, 0, 0, 0, 0, 0,
               0, 0, 0, Odom(0), Odom(1), Odom(2), 0, 0, 0,
               0, 0, 0, 0, 0, 0, Odom(0), Odom(1), Odom(2);
        A.conservativeResize(A.rows()+3, A.cols());
        A.bottomRows(3) = A_i;

        std::cout << "No. of lens: " << now_len << " / " << data_len << std::endl;
        std::cout << "Matrix A is of size " << A.rows() << "x" << A.cols() << std::endl;

        b.conservativeResize(b.rows()+3, b.cols());
        b.bottomRows(3) = scan;
        std::cout << "Vector b is of size " << b.rows() << "x" << b.cols() << std::endl;

        //end of TODO
        now_len++;
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //TODO: 求解线性最小二乘
    Eigen::VectorXd result(9);
    result = A.ldlt().solve(b);
    correct_matrix << result;
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if(now_len%data_len == 0 && now_len >= 1)
    {
        now_len = data_len;
        return true;
    }
    else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
