#include <iostream>

//#define USING_MLOGD
#include "utility/tool.h"
#include <Eigen/Core>
//#include <Eigen/Geometry>  // Eigen 几何模块


/**
 * @brief       使用Gramy-Schmidt方法实现向量正交化,并单位化
 * @param  [in]  a       向量a
 * @param  [in]  b       向量b
 * @param  [in]  c       向量c
 * @param  [out]  An      标准化后的向量A
 * @param  [out]  Bn      标准化后的向量B
 * @param  [out]  Cn      标准化后的向量C
 * @retval
 * @note  a,b,c必须为线性无关组
 */
static void schmidtOrthogonalV3D(
                        Eigen::Vector3d a,
                        Eigen::Vector3d b,
                        Eigen::Vector3d c,
                        Eigen::Vector3d* An,
                        Eigen::Vector3d* Bn,
                        Eigen::Vector3d* Cn ){

    Eigen::Vector3d A,B,C;
    //A直接赋值为a
    A = a;
    //MLOGD("vector A: %.3lf %.3f %.3f", A.x(), A.y(), A.z() );
    //求B
    double xab = A.dot(b) / A.dot( A );
    B = b - xab * A;
    //MLOGD("vector B: %.3lf %.3f %.3f", B.x(), B.y(), B.z() );
    //求C
    double xac = A.dot( c ) / A.dot( A );
    double xbc = B.dot( c ) / B.dot( B );
    C = c - ( xac * A ) - ( xbc * B );
    //MLOGD("vector C: %.3lf %.3f %.3f", C.x(), C.y(), C.z() );
    //单位化并输出
    *An = A.normalized();
    *Bn = B.normalized();
    *Cn = C.normalized();
    //MLOGD("normalized vector A: %.3lf %.3f %.3f", An->x(), An->y(), An->z() );
    //MLOGD("normalized vector B: %.3lf %.3f %.3f", Bn->x(), Bn->y(), Bn->z() );
    //MLOGD("normalized vector C: %.3lf %.3f %.3f", Cn->x(), Cn->y(), Cn->z() );
}


//--------------------测试函数----------------------------
static void schmidtOrthogonalTest( void ) {
    //线性无关组1
    Eigen::Vector3d a(1,1.2,0);
    Eigen::Vector3d b(1,2,0);
    Eigen::Vector3d c(0,1,1);

    //线性无关组2
    Eigen::Vector3d v1(9,1.2,2.4);
    Eigen::Vector3d v2(1,2,6.7);
    Eigen::Vector3d v3(5,1.5,1);

    Eigen::Vector3d A,B,C;

    schmidtOrthogonalV3D( a, b, c, &A, &B, &C );
    //schmidtOrthogonalV3D( v1, v2, v3, &A, &B, &C );
    TPLOGI("normalized vector A: %.3lf %.3f %.3f", A.x(), A.y(), A.z() );
    TPLOGI("normalized vector B: %.3lf %.3f %.3f", B.x(), B.y(), B.z() );
    TPLOGI("normalized vector C: %.3lf %.3f %.3f", C.x(), C.y(), C.z() );
}

