/** 
 *  @brief	    
 *  @details    
 *  @author     Harry-Qu
 *  @date       2022/10/26
 *  @version    1.0
 *  @par        日志
*/

#ifndef GY86_DATATYPE_H
#define GY86_DATATYPE_H

typedef struct {
    float x;
    float y;
    float z;
} vector3f_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vector3int16_t;

typedef struct {
    float a;
    float b;
    float c;
    float d;
} quat_t;   //四元数结构体

#endif //GY86_DATATYPE_H
