#ifndef _CONFIGER_HPP_
#define _CONFIGER_HPP_

#include <iostream>
#include "yaml-cpp/yaml.h"


//c++ CONFIG class  (  yaml or ini)
class Configer
{

public:
    Configer(){
        ;
    }
    void Init( std::string cfg_path  ){
        yaml_cfg_ = YAML::LoadFile( cfg_path );
    }

    /**
    * @brief  从yaml配置文件中读取数据 T is datatype: int double string
    * @param  [in] level
    * @param  [in,out]
    * @retval     读取结果
    * @note
    */
    template<typename T> T 
    GetParameter( std::string  id ){
        T ret;
        try {
            ret = yaml_cfg_[ id ].as<T>();
            std::cout<<"Found parameter: "<< id <<": "<< ret <<std::endl;
            return ret;
        } catch (std::exception e ) {
            std::cout<<"Can not find parameter: "<< id <<" program exit!" <<std::endl;
            exit(1);  //找不到直接退出
        }
    }

    /**
    * @brief  从yaml配置文件中读取数据 T is datatype: int double string
    * @param  [in] id_level_f    1级ID  必须有
    * @param  [in] id_level_n    2级ID  必须有
    * @param  [in,out]
    * @retval     读取结果
    * @note
    */
    template<typename T> T
    GetParameter( std::string id_level_f, std::string id_level_n ){
        T ret;
        try {
            ret = yaml_cfg_[ id_level_f ][ id_level_n ].as<T>();
            std::cout<<"Found parameter: "<< id_level_f + "-" + id_level_n
                    <<": "<< ret <<std::endl;
            return ret;
        } catch (std::exception e ) {
            std::cout<<"Can not find parameter: "<< id_level_f + "-" + id_level_n
                    <<" program exit!" <<std::endl;
            exit(1);  //找不到直接退出
        }
    }

    ~Configer(){
        ;
    }
private:
    YAML::Node yaml_cfg_;
};

#endif



