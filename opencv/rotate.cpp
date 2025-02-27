#include<vector>
#include <cmath>

using namespace std;
#define width   750
#define height  500


//一个点绕中心旋转角度，返回值，返回旋转后的坐标 P‘=R*(P-C)+C
vector<float> rotate(vector<int> point,vector<int> center,float degree)
{
    vector<float> result(2),temp(2);//结果和要旋转的点到中心距离的向量
    temp[0]=(float)(point[0]-center[0]);//(P-C)
    temp[1]=(float)(point[1]-center[1]);
    auto rad=degree*M_PI/180.0;
    result[0]=cos(rad)*temp[0]-sin(rad)*temp[1]+(float)center[0];
    result[1]=sin(rad)*temp[0]+cos(rad)*temp[1]+(float)center[1];
    return result;//旋转之后的坐标
}

//旋转后的坐标求解旋转前的坐标，返回旋转之前的坐标 R^{-1}*(P'-C)+C=P
vector<float> rotate_inv(vector<int> point,vector<int> center,float degree)
{
    vector<float> result(2),temp(2);//结果和要旋转的点到中心距离的向量
    temp[0]=(float)(point[0]-center[0]);//(P'-C)
    temp[1]=(float)(point[1]-center[1]);
    auto rad=degree*M_PI/180.0;
    result[0]=cos(rad)*temp[0]+sin(rad)*temp[1]+(float)center[0];
    result[1]=-sin(rad)*temp[0]+cos(rad)*temp[1]+(float)center[1];
    return result;//旋转之前的坐标
}

void rotate(float deg)
{
    vector<int> left_up(2),left_down(2),right_up(2),right_down(2);
    left_up={0,0},left_down={height,0},right_up={0,width},right_down={height,width};
    auto rotated_left_up=rotate(left_up,left_down,deg),rotated_left_down=rotate(left_down,left_down,deg),
    rotated_right_up=rotate(right_up,left_down,deg),rotated_right_down=rotate(right_down,left_down,deg);
    int h=(int)std::max(rotated_left_up[0],rotated_left_down[0],rotated_right_up[0],rotated_right_down[0]);
}

int main(int argc, char **argv)
{
    
    return 0;
}
