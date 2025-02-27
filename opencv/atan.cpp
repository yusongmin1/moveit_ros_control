
#include <iostream>
#include <cmath> // 包含数学函数的头文件

int main() {
    double sinValue = 0; // 正弦值
    double cosValue = 0.866; // 余弦值，对应于sinValue的值

    // 计算正切值
    double tanValue = sinValue / cosValue;

    // 使用atan2计算角度，这里我们传入tanValue作为y，1作为x
    double angle_in_radians = atan2(1, 0);

    std::cout << "The angle whose sine is " << sinValue << " and cosine is " << cosValue << " is " << angle_in_radians << " radians." << std::endl;

    // 将弧度转换为角度
    double angle_in_degrees = angle_in_radians * (180.0 / M_PI);

    std::cout << "The angle in degrees is " << angle_in_degrees << " degrees." << std::endl;

    return 0;
}