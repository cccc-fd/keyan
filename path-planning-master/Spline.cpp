#include "Spline.h"  // 包含Spline类的声明
#include <cstring>   // 用于字符串处理
#include <algorithm> // 用于使用max_element和min_element函数

using namespace std; // 使用标准命名空间

namespace SplineSpace // 定义SplineSpace命名空间，用于封装样条插值相关功能
{
    // 构造函数：初始化样条插值对象
    // 参数:
    // x0 - 输入点的x坐标数组
    // y0 - 输入点的y坐标数组
    // num - 输入点的数量
    // bc - 边界条件类型 (GivenFirstOrder或GivenSecondOrder)
    // leftBoundary - 左边界条件值
    // rightBoundary - 右边界条件值
    Spline::Spline(const double* x0, const double* y0, const int& num,
                   BoundaryCondition bc, const double& leftBoundary, const double& rightBoundary)
            :GivenX(x0), GivenY(y0), GivenNum(num), Bc(bc), LeftB(leftBoundary), RightB(rightBoundary)
    {
        // 检查输入参数是否有效
        if((x0==NULL) | (y0==NULL) | (num<3))
        {
            throw SplineFailure("构造失败,已知点数过少"); // 抛出异常，至少需要3个点
        }
        PartialDerivative = new double[GivenNum];	// 为二阶偏导数分配内存空间
        MaxX = *max_element(GivenX, GivenX+GivenNum); // 计算x范围的最大值
        MinX = *min_element(GivenX, GivenX+GivenNum); // 计算x范围的最小值

        // 根据边界条件类型计算偏导数
        if(Bc==GivenFirstOrder)		// I型边界条件 - 指定两端的一阶导数
            PartialDerivative1();
        else if(Bc == GivenSecondOrder)	// II型边界条件 - 指定两端的二阶导数
            PartialDerivative2();
        else
        {
            delete[] PartialDerivative; // 释放已分配的内存
            throw SplineFailure("边界条件参数错误"); // 抛出异常，边界条件类型错误
        }
    }

    // I型边界条件求偏导 - 指定两端的一阶导数
    void Spline::PartialDerivative1(void)
    {
        // 追赶法解三对角方程组求二阶偏导数
        double *a = new double[GivenNum];    // a: 三对角矩阵的下对角线
        double *b = new double[GivenNum];    // b: 三对角矩阵的主对角线
        double *c = new double[GivenNum];    // c: 三对角矩阵的上对角线
        double *d = new double[GivenNum];    // d: 方程组右端项

        double *f = new double[GivenNum];    // 存储一阶差商

        double *bt = new double[GivenNum];   // 追赶法中间变量
        double *gm = new double[GivenNum];   // 追赶法中间变量

        double *h = new double[GivenNum];    // 存储各段步长

        for(int i=0; i<GivenNum; i++)  b[i]=2;  // 三对角矩阵主对角线全部为2

        // 计算各段步长
        for(int i=0; i<GivenNum-1; i++)  h[i]=GivenX[i+1]-GivenX[i];

        // 计算下对角线系数
        for(int i=1; i<GivenNum-1; i++)  a[i]=h[i-1]/(h[i-1]+h[i]);
        a[GivenNum-1]=1;

        // 计算上对角线系数
        c[0]=1;
        for(int i=1; i<GivenNum-1; i++)  c[i]=h[i]/(h[i-1]+h[i]);

        // 计算一阶差商
        for(int i=0; i<GivenNum-1; i++)
            f[i]=(GivenY[i+1]-GivenY[i])/(GivenX[i+1]-GivenX[i]);

        // 计算方程组右端项
        d[0]=6*(f[0]-LeftB)/h[0]; // 左边界条件
        d[GivenNum-1]=6*(RightB-f[GivenNum-2])/h[GivenNum-2]; // 右边界条件

        // 计算内部节点的右端项
        for(int i=1; i<GivenNum-1; i++)  d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

        // 追赶法求解三对角方程组
        // 第一步：消元，计算bt和gm
        bt[0]=c[0]/b[0];
        for(int i=1; i<GivenNum-1; i++)  bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[0]=d[0]/b[0];
        for(int i=1; i<=GivenNum-1; i++)  gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        // 第二步：回代，求解二阶偏导数
        PartialDerivative[GivenNum-1]=gm[GivenNum-1];
        for(int i=GivenNum-2; i>=0; i--)  PartialDerivative[i]=gm[i]-bt[i]*PartialDerivative[i+1];

        // 释放临时内存
        delete[] a;
        delete[] b;
        delete[] c;
        delete[] d;
        delete[] gm;
        delete[] bt;
        delete[] f;
        delete[] h;
    }

    // II型边界条件求偏导 - 指定两端的二阶导数
    void Spline::PartialDerivative2(void)
    {
        // 追赶法解三对角方程组求二阶偏导数
        double *a = new double[GivenNum];    // a: 三对角矩阵的下对角线
        double *b = new double[GivenNum];    // b: 三对角矩阵的主对角线
        double *c = new double[GivenNum];    // c: 三对角矩阵的上对角线
        double *d = new double[GivenNum];    // d: 方程组右端项

        double *f = new double[GivenNum];    // 存储一阶差商

        double *bt = new double[GivenNum];   // 追赶法中间变量
        double *gm = new double[GivenNum];   // 追赶法中间变量

        double *h = new double[GivenNum];    // 存储各段步长

        // 初始化系数
        for(int i=0; i<GivenNum; i++)  b[i]=2;  // 三对角矩阵主对角线全部为2

        // 计算各段步长
        for(int i=0; i<GivenNum-1; i++)  h[i]=GivenX[i+1]-GivenX[i];

        // 计算下对角线系数
        for(int i=1; i<GivenNum-1; i++)  a[i]=h[i-1]/(h[i-1]+h[i]);
        a[GivenNum-1]=1;

        // 计算上对角线系数
        c[0]=1;
        for(int i=1; i<GivenNum-1; i++)  c[i]=h[i]/(h[i-1]+h[i]);

        // 计算一阶差商
        for(int i=0; i<GivenNum-1; i++)
            f[i]=(GivenY[i+1]-GivenY[i])/(GivenX[i+1]-GivenX[i]);

        // 计算内部节点的右端项
        for(int i=1; i<GivenNum-1; i++)  d[i]=6*(f[i]-f[i-1])/(h[i-1]+h[i]);

        // 考虑边界条件，修改方程
        d[1]=d[1]-a[1]*LeftB;  // 左边界条件
        d[GivenNum-2]=d[GivenNum-2]-c[GivenNum-2]*RightB;  // 右边界条件

        // 追赶法求解内部节点的二阶偏导数(不含两端点)
        bt[1]=c[1]/b[1];
        for(int i=2; i<GivenNum-2; i++)  bt[i]=c[i]/(b[i]-a[i]*bt[i-1]);

        gm[1]=d[1]/b[1];
        for(int i=2; i<=GivenNum-2; i++)  gm[i]=(d[i]-a[i]*gm[i-1])/(b[i]-a[i]*bt[i-1]);

        // 回代
        PartialDerivative[GivenNum-2]=gm[GivenNum-2];
        for(int i=GivenNum-3; i>=1; i--)  PartialDerivative[i]=gm[i]-bt[i]*PartialDerivative[i+1];

        // 设置两端点的二阶偏导数为指定值
        PartialDerivative[0]=LeftB;  // 左边界的二阶导数
        PartialDerivative[GivenNum-1]=RightB;  // 右边界的二阶导数

        // 释放临时内存
        delete[] a;
        delete[] b;
        delete[] c;
        delete[] d;
        delete[] gm;
        delete[] bt;
        delete[] f;
        delete[] h;
    }

    // 单点插值实现：计算给定x值对应的y值
    bool Spline::SinglePointInterp(const double& x, double& y) noexcept(false)
{
    // 检查x是否在插值范围内
    if((x<MinX) | (x>MaxX))
    throw SplineFailure("不支持外插值");  // 抛出异常，不支持外插值

    int klo, khi, k;
    klo=0; khi=GivenNum-1;  // 初始化区间搜索的边界
    double hh, bb, aa;

    // 二分法查找x所在的区间段[klo, khi]
    while(khi-klo>1)
{
    k=(khi+klo)>>1;  // 相当于(khi+klo)/2，但更高效
    if(GivenX[k]>x)  khi=k;
    else klo=k;
}
hh=GivenX[khi]-GivenX[klo];  // 当前区间长度

// 计算插值系数
aa=(GivenX[khi]-x)/hh;  // 左端点的权重
bb=(x-GivenX[klo])/hh;  // 右端点的权重

// 三次样条插值公式
// y = a*y_klo + b*y_khi + ((a^3-a)*M_klo + (b^3-b)*M_khi)*h^2/6
// 其中M_i是节点i处的二阶导数值
y=aa*GivenY[klo] + bb*GivenY[khi] +
  ((aa*aa*aa-aa)*PartialDerivative[klo] + (bb*bb*bb-bb)*PartialDerivative[khi]) * hh*hh/6.0;

return true;  // 插值成功
}

// 多点插值实现：计算多个x值对应的y值
bool Spline::MultiPointInterp(const double* x, const int& num, double* y) noexcept(false)
{
// 逐点调用单点插值函数
for(int i = 0; i < num; i++)
{
SinglePointInterp(x[i], y[i]);
}
return true;  // 插值成功
}

// 自动多点插值实现：在最小值和最大值之间均匀采样并插值
bool Spline::AutoInterp(const int& num, double* x, double* y) noexcept(false)
{
// 检查输出点数是否合法
if(num < 2)
throw SplineFailure("至少要输出两个点");  // 抛出异常，要求至少输出两个点

// 计算均匀步长
double perStep = (MaxX-MinX)/(num-1);

// 生成均匀分布的x值并计算对应的y值
for(int i = 0; i < num; i++)
{
x[i] = MinX+i*perStep;  // 计算当前x值
SinglePointInterp(x[i], y[i]);  // 计算对应的y值
}
return true;  // 插值成功
}

// 析构函数：释放分配的内存
Spline::~Spline()
{
    delete[] PartialDerivative;  // 释放二阶偏导数数组的内存
}

// 异常类实现
// 构造函数：初始化错误信息
SplineFailure::SplineFailure(const char* msg):Message(msg){};

// 获取错误信息的方法
const char* SplineFailure::GetMessage(){return Message;}
}