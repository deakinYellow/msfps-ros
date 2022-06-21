#ifndef MDATAFILTER_HPP
#define MDATAFILTER_HPP

#include <numeric>
#include "meigen.hpp"

namespace  mdf {


////求vector均值
static double GetVetor1dMean( const std::vector<double>& vector ){
    double sum = std::accumulate( std::begin( vector ), std::end( vector ), 0.0 );
    return sum / vector.size();
}

////求vector标准差
static double GetVetor1dStandardDeviation( const std::vector<double>& vector ){

    double sum = std::accumulate( std::begin( vector ), std::end( vector ), 0.0 );
    double mean = sum / vector.size();

    double sum_var  = 0.0;
    std::for_each( std::begin( vector ), std::end( vector ), [&](const double d ) {
        sum_var += ( d - mean ) * ( d - mean );
    });

    double sd = 0;
    if( vector.size() > 1 ){
        sd = std::sqrt( sum_var / ( vector.size() - 1 ) );
    }
    return sd;
}

////对vector排序并取中间部分求均值
///[in]  avalid_percent 有效数据百分比(取值:0-1)
static double GetVetor1dMiddleMean( std::vector<double>& vector , double avalid_percent ){
    long n = long( vector.size());
    long exclude = long( n * ( 1 - avalid_percent ) / 2 );
    std::sort( std::begin( vector ), std::end( vector ) );
    double sum = std::accumulate( std::begin( vector ) + exclude , std::end( vector ) - exclude, 0.0 );
    return sum / ( n - 2 * exclude );
}

//数据低通滤波
template< class T >
class LowPass{

public:
    LowPass( const double& weight_factor ) : weight_factor_( weight_factor ){
        ;
    }

    void SetInitialValue( const T& value ){
        last_value_ = value;
    }

    T Updata( const T& current_value , const double& dynamic_factor ){
        T ret;
        if( dynamic_factor != 0.0 ){
            ret = dynamic_factor  * current_value + ( 1 - dynamic_factor ) * last_value_;
        }
        else {
            ret = weight_factor_  * current_value + ( 1 - weight_factor_ ) * last_value_;
        }
        last_value_ = ret;
        return ret;
    }

private:
    double weight_factor_ = 0;
    T last_value_;
};

//一阶互补滤波
template< class T >
class FirstOderComplementary{

public:
    void SetInitialValue( const T& x ){
        x_ = x;
    }
    FirstOderComplementary( const double& factor ) : factor_( factor ){
        ;
    }
    // xk 为当前观测值，dx为变化量
    T Update( const T& xk, const T& dx , const double& dynamic_factor ){
        if( dynamic_factor != 0.0 ){
            x_ = dynamic_factor * xk + ( 1 - dynamic_factor ) * ( x_ + dx );
        }
        else {
            x_ = factor_ * xk + ( 1 - factor_ ) * ( x_ + dx );
        }
        return x_;
    }
private:
    double factor_;
    T  x_;
};  //end of FirstOderComplementary

//旋转低通滤波, 将一组值旋转到新的坐标系，做低通滤波，再转回原来坐标系并输出值
class RotationLowPassVector3D{

public:
    explicit RotationLowPassVector3D( const Eigen::Vector3d& factor ) {
        lpf_x_ =  new LowPass<double>( factor.x() );
        lpf_y_ =  new LowPass<double>( factor.y() );
        lpf_z_ =  new LowPass<double>( factor.z() );
    }

    void SetInitialValue( const Eigen::Vector3d& value ){
        lpf_x_->SetInitialValue( value.x() );
        lpf_y_->SetInitialValue( value.y() );
        lpf_z_->SetInitialValue( value.z() );
    }

    Eigen::Vector3d Update( const Eigen::Vector3d& current_value, const  Eigen::Quaterniond& current_q ){
        //将目标值转换到目标旋转坐标系
        rotation_value_ = meigen::Vector3dTransformByQuaterniond( current_value, current_q );

        rotation_value_.x() = lpf_x_->Updata( rotation_value_.x(), 0 );
        rotation_value_.y() = lpf_y_->Updata( rotation_value_.y(), 0 );
        rotation_value_.z() = lpf_z_->Updata( rotation_value_.z(), 0 );

        //旋转回原来坐标系
        ret_value_ = meigen::Vector3dTransformByQuaterniond( rotation_value_ , current_q.inverse() );
        return ret_value_;
    }

    ~RotationLowPassVector3D( ){
        delete lpf_x_;
        delete lpf_y_;
        delete lpf_z_;
    }

public:
    LowPass<double>*  lpf_x_;
    LowPass<double>*  lpf_y_;
    LowPass<double>*  lpf_z_;

    Eigen::Vector3d rotation_value_;
    Eigen::Vector3d ret_value_;
}; // end of RotationLowPassVector3D


}  //end of mdf

#endif   //end of MDATAFILTER_HPP


