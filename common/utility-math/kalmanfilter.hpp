/*****************************************************************
 *卡尔曼滤波器
***************************************************************/

#include <iostream>


#define  CH0_KALMAN_Q	 0.1
#define  CH0_KALMAN_R 	 100 * CH0_KALMAN_Q
/*---------------------------------------------------------------
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
*/

namespace kf {

//单变量卡尔曼滤波器,状态转移矩阵和观测矩阵均为1
//相当于低通滤波器,具有时延,对于理解卡尔曼滤波器有帮助，实际使用价值不大
//对单个double型数据进行滤波,一个对象只能对应一个数据对象,不能用于多个数据对象
class  SingleValueLowPass
{
public:
        double Update( const double resrc_data,
                   const double Q,
                   const double R  ) {

            double x_mid = sf_x_last_;
            double x_now;

            double p_mid ;
            double p_now;
            double K;

            //预测过程
            x_mid = sf_x_last_; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
            p_mid = sf_p_last_ + Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
            //更新过程
            K = p_mid / (p_mid+R); //计算卡尔曼增益,R为噪声
            x_now = x_mid + K * ( resrc_data - x_mid );//更新状态值（最优估计）
            p_now = (1-K) * p_mid;// 最优值对应的covariance 协方差
            //状态保存
            sf_p_last_ = p_now;
            sf_x_last_ = x_now;
            return x_now;
    }

private:
    double sf_x_last_;
    double sf_p_last_;

};


//Eigen Vector3d LowPass
class  Vector3dLowPass{
};



}


