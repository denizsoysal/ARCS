#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "test_struct.h"
#include <read_file/read_file.h>

int read_params_x(param_x_t* in)
{
    int i = 0;  // Keeps tracks of amount of elements inside param_array

    /* define array */
    int length_array = 15;  // Amount of parameters to be read, set by user
    param_array_t param_array[length_array];
    
    /* User input where to write data to */
    param_array[i] = (param_array_t) {"a1", &(in->a1), PARAM_TYPE_INT};     i++;
    param_array[i] = (param_array_t) {"a2", &(in->a2), PARAM_TYPE_INT};     i++;
    param_array[i] = (param_array_t) {"a3", &(in->a3), PARAM_TYPE_INT};     i++;
    param_array[i] = (param_array_t) {"b1", &(in->b1), PARAM_TYPE_FLOAT};   i++;
    param_array[i] = (param_array_t) {"b2", &(in->b2), PARAM_TYPE_FLOAT};   i++;
    param_array[i] = (param_array_t) {"b3", &(in->b3), PARAM_TYPE_FLOAT};   i++;
    param_array[i] = (param_array_t) {"c1", in->c1, PARAM_TYPE_CHAR};       i++;
    param_array[i] = (param_array_t) {"c2", in->c2, PARAM_TYPE_CHAR};       i++;
    param_array[i] = (param_array_t) {"c3", in->c3, PARAM_TYPE_CHAR};       i++;
    param_array[i] = (param_array_t) {"array/d1", &(in->array->d1), PARAM_TYPE_INT};    i++;
    param_array[i] = (param_array_t) {"array/d2", &(in->array->d2), PARAM_TYPE_FLOAT};  i++;
    param_array[i] = (param_array_t) {"array/d3", in->array->d3, PARAM_TYPE_CHAR};      i++;
    param_array[i] = (param_array_t) {"array/arrayception/e1", &(in->array->arrayception->e1), PARAM_TYPE_INT};     i++;
    param_array[i] = (param_array_t) {"array/arrayception/e2", &(in->array->arrayception->e2), PARAM_TYPE_FLOAT};   i++;
    param_array[i] = (param_array_t) {"array/arrayception/e3", in->array->arrayception->e3, PARAM_TYPE_CHAR};       i++;

    /* Copy paste generic reader function */
    int status;
    read_from_input_file(in->param_file_path, param_array,i, &status);
}